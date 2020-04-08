#include <iostream>
using namespace std;
#include "gtest/gtest.h"

#include "ClippingDistortionBlock.hpp"

#define NUM_SAMPLES 1024
static sample_t testBuf[NUM_SAMPLES];

static void fillTestBufferWithRampFunction(sample_t * buf){
  for(uint32_t i=0; i<NUM_SAMPLES; i++){
    buf[i] = (sample_t)(-0x8000 + ((float)i*0x10000)/NUM_SAMPLES);
  }
}


TEST(ClippingDistortionBlock, clippingdistortion_setClipFactor_outputIsClipped)
{
  ClippingDistortionBlock block("name", NUM_SAMPLES, ClippingDistortionBlock::CLIPPING_TYPE_HARD);

  float clippingFactor = 0.8;
  block.setClippingFactor(clippingFactor);
  sample_t max = 0x8000 * clippingFactor;
  sample_t min = -1*0x8000 * clippingFactor;

  fillTestBufferWithRampFunction(testBuf);

  block.process(testBuf);
  sample_t * out = block.getOutputBuffer();

  //ASSERT_EQ(0x4000, 0x8000 * clippingFactor);

  for(uint32_t i=0; i<NUM_SAMPLES; i++){
    ASSERT_LE(out[i], 0x8000 * clippingFactor);
    ASSERT_GE(out[i], -1 * 0x8000 * clippingFactor);
  }
}


TEST(ClippingDistortionBlock, SoftClipping_setClipFactor_OutputIsClipped)
{
  ClippingDistortionBlock block("name", NUM_SAMPLES, ClippingDistortionBlock::CLIPPING_TYPE_SOFT);

  float clippingFactor = 0.8;
  block.setClippingFactor(clippingFactor);
  sample_t max = 0x8000 * clippingFactor;
  sample_t min = -1*0x8000 * clippingFactor;

  fillTestBufferWithRampFunction(testBuf);

  block.process(testBuf);
  sample_t * out = block.getOutputBuffer();

  //ASSERT_EQ(0x4000, 0x8000 * clippingFactor);

  for(uint32_t i=0; i<NUM_SAMPLES; i++){
    ASSERT_LE(out[i], 0x8000 * clippingFactor);
    ASSERT_GE(out[i], -1 * 0x8000 * clippingFactor);

    //printf("out[%d] = %f\n", i, out[i]);
  }
}

TEST(ClippingDistortionBlock, SoftClipping_WithinThreshold_AppliesWaveShapingFunction)
{
  ClippingDistortionBlock block("name", NUM_SAMPLES, ClippingDistortionBlock::CLIPPING_TYPE_SOFT);

  fillTestBufferWithRampFunction(testBuf);

  block.process(testBuf);
  sample_t * out = block.getOutputBuffer();

  // shaping function
  //sample_t testSample = testBuf[500]/32000.0; //normalize to 1
  //sample_t expectedOutput = (testSample - (testSample*testSample*testSample / 3)) * 32000.0;
  //ASSERT_FLOAT_EQ(out[500], expectedOutput);

  // clip .. how to clip?
  //sample_t sampleExpectedToBeClipped = out[1000];
  //sample_t expectedClippedValue = (sample_t)0x8000  * (sample_t) 2 / (sample_t) 3;
  //ASSERT_FLOAT_EQ(sampleExpectedToBeClipped, expectedClippedValue);
}



TEST(ClippingDistortionBlock, FloatingPointSanityCheck)
{
  float sample1 = (float)3.14;
  printf("sample1 = %f\n", sample1);
  printf("sample1 = %a\n", sample1);
  uint32_t i = 100;
  printf("wtf = %f\n", (sample_t)(-0x8000 + ((float)i*0x10000)/NUM_SAMPLES));
  sample_t expectedClippedValue = (sample_t)0x8000 * (sample_t)2 / (sample_t) 3;
  printf("expectedClippedValue = %f\n", expectedClippedValue);
 
}