#include "ProcessBlock.hpp"

class GainBlock : public ProcessBlock{
  float gain;
  float gainFactor;
public:
  GainBlock(const char * name, uint32_t size) :
    ProcessBlock(name, size){
    gain = 1.0;
    gainFactor = 8.0;
  }

  void setGainFactor(float factor){
    gainFactor = factor;
  }
  char * paramToString(BlockParamIdentifier_t id){
    (void)id;
    static char str[10];
    int gainFirstDigit = (int)gain;
    tfp_snprintf(str, 10, "%d", gainFirstDigit);
    float gainFirstDecimalfloat = (gain - gainFirstDigit) * 10.0;
    int gainFirstDecimal = int(gainFirstDecimalfloat);
    str[1] = '.';
    tfp_snprintf(&str[2], 7, "%d", gainFirstDecimal);
    str[9] = 0;
    return str;
  }

  void setMIDIParameter(BlockParamIdentifier_t id, int value){
    (void)id;

    gain = (float)value * gainFactor /128.0;

    static char str[100];
    int size = tfp_snprintf(str,100, "%s, Gain, %s\n", name, paramToString(id));
    SerialLogger_Log(LOGTYPE_BLOCKGRAPH_NODE_UPDATE, (uint8_t *)str, size);
  }

  void process(sample_t * samplesToProcess)
  {
    for(uint32_t i=0; i<num_samples; i++){
      inputBuffer[i] = samplesToProcess[i];
    }
    for(uint32_t i=0; i<num_samples; i++){
      outputBuffer[i] = inputBuffer[i] * gain;
    }
  }
};

class ClippingDistortionBlock : public ProcessBlock{
  float clipping_percent;
public:
  ClippingDistortionBlock(const char * name, uint32_t size) :
    ProcessBlock(name, size){
    clipping_percent = 0.5;
  }

  void process(sample_t * samplesToProcess)
  {
    for(uint32_t i=0; i<num_samples; i++){
      inputBuffer[i] = samplesToProcess[i];
    }

    //clip half way, both ends
    sample_t max = (sample_t) ((float)0x8000 * (clipping_percent));
    sample_t min = (sample_t) ((float) -0x8000 * (clipping_percent));

    for(uint32_t i=0; i<num_samples; i++){
      if(inputBuffer[i] > max)
        outputBuffer[i] = max;
      else if(inputBuffer[i] < min)
        outputBuffer[i] = min;
      else
        outputBuffer[i] = inputBuffer[i];
    }
  }

  void setMIDIParameter(BlockParamIdentifier_t id, int value){
    (void)id;

    clipping_percent = ((float)value/128.0);
    int clipping_percent_int = (int) (clipping_percent*100.0);
    static char str[100];
    int size = tfp_snprintf(str,100, "%s, Clip(%%), %d\n", name, clipping_percent_int);
    SerialLogger_Log(LOGTYPE_BLOCKGRAPH_NODE_UPDATE, (uint8_t *)str, size);
  }
};