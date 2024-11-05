
## 給速傳權限
使用 ```lsusb``` 命令： 這個命令會列出所有連接的 USB 設備及其基本資訊，包括 idVendor 和 idProduct。


```
lsusb
```

執行後，會看到類似以下的輸出：


```
Bus 001 Device 002: ID 1234:5678 Example USB Device
```
其中 1234 是 idVendor（廠商 ID），5678 是 idProduct（產品 ID）。


確認找到的 idVendor 和 idProduct： 一旦找到設備的 idVendor 和 idProduct 值，就可以在 udev 規則中使用這些值來創建固定名稱。

完整示範
假設您找到的 idVendor 是 1234，idProduct 是 5678，則可以創建 udev 規則來將設備綁定到固定名稱 /dev/drone_usb：

```
sudo nano /etc/udev/rules.d/99-drone-usb.rules
```

在文件中添加以下內容：

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", SYMLINK+="drone_usb", MODE="0666"
```

然後，重啟 udev 服務並觸發規則：

```
sudo udevadm control --reload-rules
sudo udevadm trigger
```


這樣，每次插入該 USB 設備，系統都會將它綁定到 /dev/drone_usb，無論它實際分配到的名稱是 /dev/ttyUSB0 還是 /dev/ttyUSB1