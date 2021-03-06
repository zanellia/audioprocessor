#include "ProcessBlock.hpp"
extern "C"{
#include "MemoryLogger.h"
#include "SerialLogger.h"
#include "tinyprintf.h"
}

class ClippingDistortionBlock : public ProcessBlock{

public:
  enum ClippingType {
                     CLIPPING_TYPE_SOFT,
                     CLIPPING_TYPE_HARD
  };

private:
  sample_t clipping_percent;
  ClippingType clipping_type;

public:


  ClippingDistortionBlock(const char * name, uint32_t size, ClippingType clippingType) :
    ProcessBlock(name, size){
    clipping_percent = 0.5;
    clipping_type = clippingType;
  }

  void setClippingFactor(sample_t factor){
    clipping_percent = factor;
  }

  void process(const sample_t *samplesToProcess)
  {
    for(uint32_t i=0; i<num_samples; i++){
      inputBuffer[i] = samplesToProcess[i];
    }

    if(clipping_type == CLIPPING_TYPE_HARD)
      clip_hard();
    else
      clip_soft();
  }

  void clip_hard(void)
  {
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

  void clip_soft(void)
  {
    //Normalize and Waveshape

    //This formula assumes the signal range is -1 to 1
    //the actual input is huge, 32000.0.
    //input has to be normalized to 1 because of the squaring term.
    //100% divide by 32000 and there will be clipping at 32000.
    //50% divide by 16000

    sample_t normalization_factor = clipping_percent * 32000.0;
    // NORMALIZE
    for(uint32_t i=0; i<num_samples; i++) {
      inputBuffer[i] = inputBuffer[i]/normalization_factor;
    }

  // USING THE DAFX Formula
    for(uint32_t i=0; i<num_samples; i++){
        if(my_abs(inputBuffer[i]) < 0.33){
          outputBuffer[i] = inputBuffer[i] * 2;
        }
        else if(my_abs(inputBuffer[i]) < 0.66){
          if(inputBuffer[i] < 0){
            sample_t x = my_abs(inputBuffer[i]);
            outputBuffer[i]=
                    (-1) *
                    (3.0 - ((2 - (3.0*x)) * (2 - (3.0*x)))) / 3.0;
          }
          else if(inputBuffer[i] > 0){
            sample_t x = inputBuffer[i];
            outputBuffer[i] =
                    (3.0 - ((2 - (3.0*x)) * (2 - (3.0*x)))) / 3.0;
          }

        }
        else {
          if(inputBuffer[i] < 0)
            outputBuffer[i] = -1;
          else if(inputBuffer[i] > 0){
            outputBuffer[i] = 1;
          }
          
        }
    }

    // DENORMALIZE
    for(uint32_t i=0; i<num_samples; i++) {
      outputBuffer[i] = outputBuffer[i]*normalization_factor;
    }

  //this is from somehwere else
    //for(uint32_t i=0; i<num_samples; i++){
    //  sample_t normalized = inputBuffer[i]/32000.0;  //normalize to 1 so the cubic works
    //  sample_t shaped = normalized - (normalized*normalized *normalized / 3);
    //  sample_t denormalized = shaped * 32000.0;
    //  outputBuffer[i] = denormalized;
    //}
  }


  void setMIDIParameter(BlockParamIdentifier_t id, int value){
    (void)id;

    clipping_percent = ((sample_t)value/128.0);

    int clipping_percent_int = (int) (clipping_percent*100.0);
    static char str[100];
    int size = tfp_snprintf(str,100, "%s, Clip(%%), %d\n", name, clipping_percent_int);
    SerialLogger_Log(LOGTYPE_BLOCKGRAPH_NODE_UPDATE, (uint8_t *)str, size);
  }


  //lol portability
  sample_t my_abs(sample_t in){
    if(in < 0){
      return in*-1.0;
    }
    return in;
  }
};
