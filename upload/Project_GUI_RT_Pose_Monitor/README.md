# display

![](https://hackmd.io/_uploads/ByAV5vuKq.png)

使用 C++/CLI 來做 Displayer，分為以下部分：

1. Serial Port 接 UART 資料：讀取目前電腦所接的 COM 並連接，淺綠色方塊顯示接到多少筆資料。

2. Body Orientation Displayer：將即時接進來的 Euler angle，建立方向餘弦矩陣 (direction cosine matrix, DCM) 來旋轉方塊，紅色 x 軸(面部朝前)，綠 y 軸(右手平舉方向)，藍 z 軸朝向上方(頭頂)。

3. 圖表：即時顯示 300 筆 roll、pitch、yaw angle 的資料。

若有發生撞擊，則黑色視窗部分會呈現紅色並通報。
