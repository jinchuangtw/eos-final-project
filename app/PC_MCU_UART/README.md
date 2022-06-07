# PC_MCU_UART
05-10 2022

## 專案結構
```
PC_MCU_UART
├─ app
│  └─ PC_MCU_UART.exe
├─ arduino
├─ CMakeLists.txt
├─ include
│  └─ UART
│     ├─ README.md
│     ├─ RS232
│     │  ├─ examples
│     │  │  ├─ demo_rx.c
│     │  │  └─ demo_tx.c
│     │  ├─ README.md
│     │  ├─ rs232.c
│     │  └─ rs232.h
│     └─ UART.h
├─ lib
│  ├─ include
│  ├─ liba
│  └─ libdll
├─ README.md
└─ src
   └─ main.cpp
```

## 說明
* 這是一份用於跟 MCU 進行 UART 通訊的專案。最後產生的可執行檔將會占用電腦的某個序列埠，定期地接收並解碼所傳入的資料。也就是說，主程式功能有二：
    * 定期接收序列埠資料 (此部分由 `RS232` 函式庫所完成)
    * 序列埠資料的解碼 (根據事先溝通好的協議，進行二進制資料的解碼)
* 而傳送至序列埠的資料，則是由嵌入式系統端傳出。控制 MCU 發送資料的程式碼請見 `arduino/MCU_UART_Tx` 路徑下的 platformIO 專案。
* PC (Rx) <-----> MCU (Tx)

## 待完成
* 與 GUI 介面的橋接
