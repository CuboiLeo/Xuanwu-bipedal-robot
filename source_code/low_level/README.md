
# This folder is for the STM32H723 dev board

## IMPORTANT - If the project is regenerated using STM32CubeMX
You will see many errors if you rebuild the project in Keil, this is because STM32CubeMX uses RVDS for freeRTOS. To fix this, follow the following steps
1. Copy the GCC folder at the current directory
2. Paste it under ./Xuanwu_bipedal_robot/Middlewares/Third_Party/FreeRTOS/Source/portable, where you will see the STM32CubeMX generated RVDS and MemMang folders
3. Go to Keil, on the project tree on the left, find the last port.c file under Middlewares/FreeRTOS folder, right click and remove it
4. Go to Options for Target (magic wand icon in the tools menu), under C/C++, remove the RVDS/ARM_CM4F folder from the Include Paths
5. Build the project, there should (hopefully) be no more errors
