
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define U_Batt_Pin GPIO_PIN_0
#define U_Batt_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_1
#define ENABLE_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_3
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_4
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_5
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_6
#define PWM4_GPIO_Port GPIOA
#define LED_Mode_Pin GPIO_PIN_11
#define LED_Mode_GPIO_Port GPIOA
#define LED_Error_Pin GPIO_PIN_12
#define LED_Error_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define Button_Pin GPIO_PIN_3
#define Button_GPIO_Port GPIOB
#define Button_EXTI_IRQn EXTI3_IRQn
#define INT_Pin GPIO_PIN_4
#define INT_GPIO_Port GPIOB
#define INT_EXTI_IRQn EXTI4_IRQn
#define AD0_Pin GPIO_PIN_5
#define AD0_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
