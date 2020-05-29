# WAM实时遥操作
UDP接收来自服务器的关节角指令，并驱动WAM执行指令

## 记录
* ``#Q1``：访问WAM远端服务器执行任意控制指令时都会抛出``异常``
* ``#A1``: 是发生在``bt-wam-zerocal``和``bt-wam-gravitycal``期间突然中止。
> CSDN [解决方法``LINK``](https://blog.csdn.net/sinat_31538869/article/details/106328075)

## v1：WAM_UDP_client
UDP接收来自服务器的关节角指令，并驱动WAM执行指令
## ``#1: wam.moveTo``
使用``moveTo``执行过慢，使得遥操作不能够实时，延迟以秒记。

## ex06采用system关节角控制画圆
### 测试加速画圆
* 通过Rate Limiter ``jp_rl``限制关节角速度为2m/s
讲
* 修改``omega``为2
```c++
	virtual void operate() {
		theta = omega * this->input.getValue();

		jp[i1] = amp * std::sin(theta) + jp_0[i1];
		jp[i2] = amp * (std::cos(theta) - 1.0) + jp_0[i2];

		this->outputValue->setData(&jp);
	}

```
### 修改有效，画圆加速

## v2:wam_udp_jp_realtime
基于ex06编写了wam_udp_jp_realtime.cpp
* 使用子线程接收udp传送的关节角
* 主线程执行system move
* 根据实际WAM的抖动情况调节了rLimit
```c++
const double rLimit[] = {0.1, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5};// 20200527 
```