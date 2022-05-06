# Hardware Config Initialization

徵求好心人幫我把圖切一切
做完覺得好長想切

## Open Project
請按照圖中的方法選板子，這次直接選了 F411 ，但我覺得按照之前一樣選 F407 好像也可以。
![Board_Selector](https://user-images.githubusercontent.com/91120147/167119615-00bf87aa-2f3f-4231-b5f4-125cbd791b36.png)

## .ioc 檔初始化 
### System Core
* System Core > NVIC > Priority Group > `4 bit` 

![NVIC](https://user-images.githubusercontent.com/91120147/167121658-fb0c45c2-6126-440c-b3be-f0cec272a281.png)

* System Core > SYS > Debug > `Serial Wire`
* System Core > Sys > Timebased Source > `TIM3` (我不知道要選哪一個)

![SYS](https://user-images.githubusercontent.com/91120147/167122209-c79c8152-9091-4771-be79-a381704b0f91.png)

### Connectivity
#### UART 
Connectivity > UART2 >  Mode > `Asynchronous`
![UART](https://user-images.githubusercontent.com/91120147/167122237-66221673-5626-4d5f-ada8-569447cc69ea.png)


#### I2C
Connectivity > I2C3 >  I2C > `I2C`
![I2C_Init](https://user-images.githubusercontent.com/91120147/167118933-ebb4a7a6-17e3-42e7-bcb1-586e92e8ce74.png)
