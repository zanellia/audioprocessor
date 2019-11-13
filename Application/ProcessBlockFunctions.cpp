#include "ProcessBlockFunctions.hpp"

void ProcessBlockFunctions_Identity(BlockState * state, sample_t * in, sample_t * out, uint32_t size)
{
  (void) state;(void) in; (void) out; (void) size;
}

void ProcessBlockFunctions_Gain2X(BlockState * state, sample_t * in, sample_t * out, uint32_t size)
{
  (void) state;
  for(uint32_t i=0; i<size; i++){
    out[i] = in[i] * 2;
  }
}


//getting rid of this
void ProcessBlockFunctions_GainParameterized(BlockState * state, sample_t * in, sample_t * out, uint32_t size)
{

  //params are midi values, 8bits.
  //scale to an float, 0 to 8.0
  int gain = state->getParam(PARAM_0);
  float gain_float = (float)gain * 8.0 /128.0;
  for(uint32_t i=0; i<size; i++){
    out[i] = in[i] * gain_float;
  }
}

void ProcessBlockFunctions_Attenuation(BlockState * state, sample_t * in, sample_t * out, uint32_t size)
{
  //params are midi values, up to 127
  //scale to an float, 0 to 2.0
  int gain = state->getParam(PARAM_0);
  float gain_float = (float)gain * 2.0 /128.0;
  for(uint32_t i=0; i<size; i++){
    out[i] = in[i] * gain_float;
  }
}

void ProcessBlockFunctions_ClippingDistortion(BlockState * state, sample_t * in, sample_t * out, uint32_t size)
{
  (void)state;
  //clip half way, both ends
  sample_t max = (sample_t) 0x4000/2;
  sample_t min = (sample_t) -(0x4000/2);

  for(uint32_t i=0; i<size; i++){
    if(in[i] > max)
      out[i] = max;
    else if(in[i] < min)
      out[i] = min;
    else
      out[i] = in[i];
  }
}

void ProcessBlockFunctions_Mixer(BlockState * state, sample_t * in, sample_t * out, uint32_t size)
{
  (void)state;

  for(uint32_t i=0; i<size; i++){
    out[i] = out[i] + in[i];
  }
}
