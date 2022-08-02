# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)


このリポジトリは主にCrazyflie 2.x の内部プログラムのソースコードを管理している，<br>
編集する頻度が高いのは /src/modules ディレクトリであり内部コントローラーを扱っている．<br>
公式 [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware)

## Compiling

編集した c ファイルのコンパイル方法
```bash
make PLATFORM=cf2
```
