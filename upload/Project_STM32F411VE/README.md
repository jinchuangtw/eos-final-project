# Project_STM32F411VE

以下將簡單說明專案內部所設計的 tasks。

## vTask_AHRS()
負責利用 `I2C` 協議跟 `MPU9255` 通訊，獲取 9-DoF 的運動資訊。在此基礎上，所開發的 AHRS 演算法將進行物體實時的姿態解算，作為安全偵測演算法的必要資訊。


## vTask_UART()
主要負責將以下的資料結構透過 UART 傳至 WiFi device。
```c=
typedef struct DataPackage
{
    double qua[4];
    double acc[3];
    int status;
} Data_t;
```
## vTask_Fall()
此 task 內包括狀態偵測與倒地偵測兩部分。首先，狀態偵測會依據 AHRS 計算出的 roll, pitch, yaw 與三軸加速度，判斷以下各狀態是否為真。

1. 大角度偵測 / per 0.5 sec：如果頭盔在 0.5 秒內，pitch 或 roll 改變超過 60 度，則我們說頭盔正在劇烈搖晃，程式內稱為RocknRoll。
2. 靜止偵測 / per 8 sec：那如果頭盔在 8 秒內的平均角度變化量小於 1.5 度，則稱之為 Sleep 狀態。
3. 撞擊偵測 / per 0.01 sec：如果頭盔在某個瞬間的三軸加速度 norm 值大於 5 倍的重力加速度，則我們會說頭盔受到 Impact。

接著，再根據這 3 個狀態 flag，確認現在是否倒地。

施工上造成的碰撞、走動、靜止必須跟異常的撞擊區分開來，是以會需要結合分析姿態才能準確判斷是否跌落。

## Future Work
* 納入高度計、WEB/APP，蜂鳴器，硬體機構輕量化等。
