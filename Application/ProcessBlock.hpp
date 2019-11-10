#ifndef __PROCESSBLOCK_HPP__
#define __PROCESSBLOCK_HPP__

#include <stdint.h>
#include "SamplingTypes.hpp"
#include "BlockState.hpp"
#include "ProcessBlockFunctions_FIRFilters.hpp"
#include "ProcessBlockFunctions.hpp"
#include "MIDI_Message.h"

extern "C"{
#include "Monitor_Task.h"
}


typedef void (* ProcessBlockFunctionPointer)(BlockState *, sample_t *, sample_t *, uint32_t);

class MIDIAssignment{
public:
  MIDI_Message_t msg;
  BlockParamIdentifier_t paramId;
};

class ProcessBlock{
public:
  virtual void process(sample_t * samplesToProcess) = 0;
  virtual void MIDIMessageReceived(MIDI_Message_t & msg) = 0;
  virtual sample_t * getOutputBuffer(void) = 0;
  virtual ~ProcessBlock() {};
};

class RealProcessBlock : public ProcessBlock{

protected:
  ProcessBlockFunctionPointer processFunc;
  sample_t * inputBuffer;
  sample_t * outputBuffer;
  uint32_t num_samples;

  BlockState * blockState;
  MIDIAssignment midiAssignments[16];  //hard coded
  int midiAssignmentIndex = 0;

public:
  RealProcessBlock(ProcessBlockFunctionPointer func, uint32_t size){
    processFunc = func;
    num_samples = size;
    inputBuffer = new sample_t[size];
    outputBuffer = new sample_t[size];

    blockState = new BlockState();
  }

  ~RealProcessBlock(){
    delete []inputBuffer;
    delete []outputBuffer;
  }

  sample_t * getOutputBuffer(void){
    return outputBuffer;
  }

  virtual void setMIDIParameter(BlockParamIdentifier_t id, int value){
    blockState -> setParam(id, value);
  }

  BlockState * getBlockState(void){return blockState;}

  void process(sample_t * samplesToProcess){
    for(uint32_t i=0; i<num_samples; i++){
      inputBuffer[i] = samplesToProcess[i];
    }

    processFunc(blockState, inputBuffer, outputBuffer, num_samples);
  }

  void MIDIMessageReceived(MIDI_Message_t & msg){
    for(int i=0; i<midiAssignmentIndex; i++){
      if(midiAssignments[i].msg.type != msg.type)
        continue;
      if(midiAssignments[i].msg.id != msg.id)
        continue;

      setMIDIParameter(midiAssignments[i].paramId, msg.value);
    }
  }

  void assignMIDIMessageToParameter(MIDI_Message_t & msg, BlockParamIdentifier_t id){
    midiAssignments[midiAssignmentIndex].msg = msg;
    midiAssignments[midiAssignmentIndex].paramId = id;
    midiAssignmentIndex++;
  }
};



class GainBlock : public RealProcessBlock{
public:
  GainBlock(uint32_t size) : RealProcessBlock(ProcessBlockFunctions_GainParameterized, size){
  }
};

class ClippingDistortionBlock : public RealProcessBlock{
public:
  ClippingDistortionBlock(uint32_t size) : RealProcessBlock(ProcessBlockFunctions_ClippingDistortion, size){
  }
};

class MixerBlock : public RealProcessBlock {
public:
  MixerBlock(uint32_t size) : RealProcessBlock(ProcessBlockFunctions_Mixer, size){
    reset();
  }

  //clear output buf
  void reset(void){
    for(uint32_t i=0; i<num_samples; i++){
      outputBuffer[i] = (sample_t) 0;
    }
  }
};


class DelayBlock : public RealProcessBlock {

  int delayNumSamples;
  DelayBuffer * delayBuffer;

public:
  DelayBlock(uint32_t size) : RealProcessBlock(ProcessBlockFunctions_Identity, size){
    delayBuffer = new DelayBuffer(1024*10);//static 10k
    delayNumSamples = 0;
  }

  //for testing
  int getDelayNumSamples(void){return delayNumSamples;}

  void setMIDIParameter(BlockParamIdentifier_t id, int value){
    (void)id;
    float delayNumSamples_float = ((float)value/128.0) * 100.0 * (1.0/1000.0) * (48000.0);
    delayNumSamples = (int) delayNumSamples_float;
    Monitor_LogLiteralString(LOGTYPE_EVENT, "DELAY VALUE UPDATED\n");
  }


  void process(sample_t * samplesToProcess)
  {
    for(uint32_t i=0; i<num_samples; i++){
      inputBuffer[i] = samplesToProcess[i];
    }

    //add delayed input to input
    //add sample to delayBuffer
    for(uint32_t i=0; i<num_samples; i++){
      outputBuffer[i] = inputBuffer[i] + delayBuffer->getDelayedSample(delayNumSamples);
      delayBuffer->insertSample(inputBuffer[i]);
    }
  }
};


#endif
