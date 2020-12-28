# STM32F746 Discovery Kit: Audio Processor

Adapted from https://github.com/bootladder/audioprocessor.

Note: Just in case someone else finds this useful, I figured that a simple pass-through can be obtained adapting https://github.com/STMicroelectronics/STM32CubeF7/blob/master/Projects/STM32746G-Discovery/Examples/BSP/Src/audio_loopback.c from the STM32Cube repo. It mostly requires changing this line https://github.com/STMicroelectronics/STM32CubeF7/blob/master/Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_audio.c#L893 such that it allows `INPUT_DEVICE_INPUT_LINE_1` too and fix the gain of the wm8994 codec here https://github.com/STMicroelectronics/STM32CubeF7/blob/master/Drivers/BSP/Components/wm8994/wm8994.c#L310 from `0x0035` to `0x0025` (both right and left channel).
