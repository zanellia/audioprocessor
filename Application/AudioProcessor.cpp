extern "C" {
#include "AudioProcessor.h"  //Audio_Task uses this interface to pass control to AudioProcessor
}

#include "SamplingTypes.h"
#include "BSP_Audio_Buffer_Interface.h"
#include "ProcessBlock.hpp"
#include "FIRBlock.hpp"
#include "ARMDSPFIRProcessor.hpp"
#include "OscillatorBlock.hpp"
#include "FFTBlock.hpp"
#include "ARMDSPFFTProcessor.hpp"
#include "MIDIMap.hpp"
#include "MIDIMessageHandler.hpp"
#include "BlockGraph.hpp"
#include "SuperSimpleProcessBlocks.hpp"
#include "ClippingDistortionBlock.hpp"
#include "DelayBlock.hpp"
#include "MixerBlock.hpp"
#include "LowPassCoefficientTable.hpp"

class AudioProcessor{
  MIDIMap midiMap;
public:
  void init(void);
  sample_t * process(sample_t * sampleBuf);
};

AudioProcessor audioProcessor;


// Currently, everything is configured statically and manually.
// Eventually there will be some configuration format
// That will be parsed into the Processing Graph and the MIDI Map.

#define createBlock(blockClass, name) static blockClass name(#name,MY_PROCESSING_BUFFER_SIZE_SAMPLES);

createBlock(GainBlock               ,gain1       )
createBlock(GainBlock               ,gain2       )
createBlock(GainBlock               ,gain3       )
createBlock(MixerBlock              ,mixer       )
createBlock(DelayBlock              ,delay       )

//////////////////////////////////

static ClippingDistortionBlock clipping1("clip1", MY_PROCESSING_BUFFER_SIZE_SAMPLES, ClippingDistortionBlock::CLIPPING_TYPE_SOFT);

//////////////////////////////////

ARMDSPFIRProcessor armdspfirp;
LowPassCoefficientTable lpct;
static FIRBlock  fir1("fir1", MY_PROCESSING_BUFFER_SIZE_SAMPLES, armdspfirp, lpct);
static FIRBlock  fir2("fir2", MY_PROCESSING_BUFFER_SIZE_SAMPLES, armdspfirp, lpct);

//////////////////////////////////

static ARMDSPFFTProcessor armDSPFFTProcessor;
static FFTBlock fft1 = FFTBlock("fft1",armDSPFFTProcessor, 2*1024, MY_PROCESSING_BUFFER_SIZE_SAMPLES);

//////////////////////////////////

float return_constant_440hz(void){return (float)fft1.getSpectrumPeakFreq();}
float return_some_amplitude(void){return (float)fft1.getSpectrumPeakMagnitude();}

OscillatorBlock square1("square1", MY_PROCESSING_BUFFER_SIZE_SAMPLES,
                        OSCILLATOR_SQUARE,
                        return_constant_440hz,
                        return_some_amplitude);

__attribute__ ((unused))
static BlockGraph blockGraph = {
  .start = &gain1,
  .edges = {
  {&gain1, &square1},
  {&square1, &gain2},
  {&gain1, &gain3},
  {&gain1, &fft1},
  {&gain3, &clipping1},
  {&clipping1, &fir2},
  {&fir2, &delay},
  {&delay, &mixer},
  {&gain2, &mixer},
  {0,0}, // null terminator
  },
  .end = &mixer,
};

__attribute__ ((unused))
static BlockGraph testerGraph = {
  .start = &gain1,
  .edges = {
    {&gain1, &square1},
  },
  .end = &square1,
};


static BlockGraph & active_block_graph = blockGraph;

void AudioProcessor::init(void)
{
  //stuff that shouldn't be here
  fir1.setCutoffFrequency(10);
  fir2.setCutoffFrequency(10);



  //midi map assigns messages to blocks
  MIDI_Message_t gain1_midi_message = {MIDI_CONTROL_CHANGE,1,1};
  MIDI_Message_t gain2_midi_message = {MIDI_CONTROL_CHANGE,2,1};
  MIDI_Message_t gain3_midi_message = {MIDI_CONTROL_CHANGE,3,1};
  MIDI_Message_t clipping1_midi_message = {MIDI_CONTROL_CHANGE,4,1};
  MIDI_Message_t fir1_midi_message = {MIDI_CONTROL_CHANGE,5,1};
  MIDI_Message_t delay_midi_message = {MIDI_CONTROL_CHANGE,6,1};
  MIDI_Message_t fir2_midi_message = {MIDI_CONTROL_CHANGE,8,1};
  midiMap.addEntry(gain1_midi_message, gain1);
  midiMap.addEntry(gain2_midi_message, gain2);
  midiMap.addEntry(gain3_midi_message, gain3);
  midiMap.addEntry(clipping1_midi_message, clipping1);
  midiMap.addEntry(fir1_midi_message, fir1);
  midiMap.addEntry(fir2_midi_message, fir2);
  midiMap.addEntry(delay_midi_message, delay);

  MIDIMessageHandler_RegisterMIDIMap(midiMap);

  //block MIDI assignments assign messages to block params
  gain1.assignMIDIMessageToParameter(gain1_midi_message, PARAM_0);
  gain2.assignMIDIMessageToParameter(gain2_midi_message, PARAM_0);
  gain3.assignMIDIMessageToParameter(gain3_midi_message, PARAM_0);
  clipping1.assignMIDIMessageToParameter(clipping1_midi_message, PARAM_0);
  fir1.assignMIDIMessageToParameter(fir1_midi_message, PARAM_0);
  fir2.assignMIDIMessageToParameter(fir2_midi_message, PARAM_0);
  delay.assignMIDIMessageToParameter(delay_midi_message, PARAM_0);
}

sample_t * AudioProcessor::process(sample_t * sampleBuf)
{
  // call reset() on all of the blocks (particularly to reset the mixers' accumulators)
  mixer.reset();


  active_block_graph.start->process(sampleBuf);

  auto edges = active_block_graph.edges;
  int i=0;
  while(edges[i].block != 0){
    edges[i].next->process(edges[i].block->getOutputBuffer());
    i++;
  }

  return active_block_graph.end->getOutputBuffer();
}

///////////////////////////////////////////////////////////////////////////////////

// This is called by the FreeRTOS Audio Task, every time the DMA receive buf fills up.
// Takes int16_t samples, processes, returns int16_t samples.
// The processor takes sample_t's, currently defined to be floats

// The AudioProcessor MUST be initialized!!
// Make a call to AudioProcessor_Init() !!!
extern "C"
int16_t *
AudioProcessor_ProcessSampleBuffer(int16_t * sampleBuf, uint32_t num_samples)
{
  static sample_t inputSampleBuffer[MY_PROCESSING_BUFFER_SIZE_SAMPLES];  //max size
  static int16_t outputInt16Buffer[MY_PROCESSING_BUFFER_SIZE_SAMPLES];

  for(uint32_t i=0; i<num_samples; i++){
    inputSampleBuffer[i] = (sample_t)sampleBuf[i];
  }

  sample_t * out = audioProcessor.process(inputSampleBuffer);

  for(uint32_t i=0; i<num_samples; i++){
    outputInt16Buffer[i] = (int16_t)out[i];
  }

  return outputInt16Buffer;
}


// This is called in the BSP_Audio_Task init.
// Could do a constructor but at this point I want the init to execute later.
// For example, the UARTs won't be initialized when the global constructors run.
extern "C" void AudioProcessor_Init(void)
{
  audioProcessor.init();
}


// Called by the monitor task to print out the edge list of the active block graph
extern "C" char * AudioProcessor_GetActiveBlockGraphEdgeListToString(void)
{
  return active_block_graph.toEdgeListJSONString();
}

// Called by the monitor task
extern "C" sample_t * AudioProcessor_GetFFTSpectrum(void)
{
  return fft1.getSpectrum();
}
