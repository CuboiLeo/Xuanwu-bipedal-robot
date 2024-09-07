#include "main.h"
#include "Test.h"
#include "FreeRTOS.h"
#include "task.h"
 
class LED_Class
{
    public:
//        LED_Class(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)\
//    :GPIOx(GPIOx),GPIO_Pin(GPIO_Pin){};
        LED_Class(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
        {
            this->GPIOx = GPIOx;
            this->GPIO_Pin = GPIO_Pin;
//            MX_GPIO_Init();
        }
 
        void open()
        {
            HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
        }
        void close()
        {
            HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
        }        
        void tiggle()
        {
            HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
        }
    private:
        GPIO_TypeDef *GPIOx;
        uint16_t GPIO_Pin;
    
};
