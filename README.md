# WAM实时遥操作
UDP接收来自服务器的关节角指令，并驱动WAM执行指令

## 一、记录
* ``#Q1``：访问WAM远端服务器执行任意控制指令时都会抛出``异常``
* ``#A1``: 是发生在``bt-wam-zerocal``和``bt-wam-gravitycal``期间突然中止。
> CSDN [解决方法``LINK``](https://blog.csdn.net/sinat_31538869/article/details/106328075)
## 二、WAM实时控制（不含Bhand）
* v1: WAM_UDP_client
* v2: wam_udp_jp_realtime
### 2.1 v1：WAM_UDP_client
UDP接收来自服务器的关节角指令，并驱动WAM执行指令
### 2.2 ``#1: wam.moveTo``
使用``moveTo``执行过慢，使得遥操作不能够实时，延迟以秒记。

### 2.3 ex06采用system关节角控制画圆
#### 2.3.1 测试加速画圆
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
#### 2.3.2 修改有效，画圆加速

### 2.4 v2:wam_udp_jp_realtime
基于ex06编写了wam_udp_jp_realtime.cpp
* 使用子线程接收udp传送的关节角
* 主线程执行system move
* 根据实际WAM的抖动情况调节了rLimit
```c++
const double rLimit[] = {0.1, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5};// 20200527 
```
## 三、Bhand & WAM实时控制
### 3.1 BHand Library
```c++
Hand& hand = *pm.getHand();// 实例hand
```
#### 3.1.1 hand.InitHand()
```c++
int InitHand(char * motor)
/*
Arguments: 
	motor:  Specifies motors to initialize.  
		motor = "1" Finger F1 "2" Finger F2 "3" Finger F3
		motor = "4" or "S" Spread Motion 扩展运动
		motor = "G" Fingers F1, F2 and F3
		motor = "" all motors
	value: Specifies the encoder position to be moved to. 
Purpose: 
	Determines encoder and motor alignment for commutation, moves all fingers and spread to open
	positions and resets the baud rate to 9600.  
Note:
	InitHand() needs to be called after the hand has been reset. This command must be run
	before any other motor commands, once the hand is turned on. 
*/
```
#### 3.1.2 hand.open()
```c++
// windos c++
int Close(char* motor)
/*
Arguments: 
	motor: Specifies which motors will be opened.  
		motor = "1" Finger F1 "2" Finger F2 "3" Finger F3
		motor = "4" or "S" Spread Motion 扩展运动
		motor = "G" Fingers F1, F2 and F3
		motor = "" all motors
Purpose: 
	Commands the selected motor(s) to move finger(s) in open direction with a velocity ramp-down
	at target limit, OT. 
*/
```
#### 3.1.3 hand.close()
```c++
// windos c++
int Close(char* motor)
/*
Purpose: Commands the selected motor(s) to move finger(s) in close direction with a velocity ramp-down
to target limit, CT. 
Note: Finger(s) close until the joint stop(s) are reached, the close target is reached, or an obstacle is
encountered
Example:
	motor = "1" Finger F1 "2" Finger F2 "3" Finger F3
	motor = "4" or "S" Spread Motion 扩展运动
	motor = "G" Fingers F1, F2 and F3
	motor = "" all motors

*/
```
#### 3.1.4 hand.GoToDifferentPositions()
```c++
int GoToDifferentPositions( int value1, int value2, int
value3, int value4 )
/*
Arguments: value1,2,3,4: Specifies the encoder position for each motor respectively. 
	value[0]: F1
	value[1]: F2
	value[2]: F3
	value[3]: Spread
Purpose: Moves all motors to specified encoder positions. 
Example:
	hand.GoToDifferentPositions(2000, 3000, 4000, 1000);
	moves finger F1 to 2000, finger F2 to 3000, finger F3 to 4000 and Spread to 1000
*/
```
#### 3.1.5  hand.GoToHome()
```c++
int GoToHome()
/*
Moves all motors to the home position, driving all fingers and the spread to their full open
position. See the BarrettHand User Manual for more information on the home position.
*/
```

#### 3.1.6  hand.GoToPosition()
```c++
int GoToPosition( char* motor, int value)
/*
Arguments: 
	motor: Specifies which motors will be closed. 
		motor = "1" Finger F1 "2" Finger F2 "3" Finger F3
		motor = "4" or "S" Spread Motion 扩展运动
		motor = "G" Fingers F1, F2 and F3
		motor = "" all motors
	value: Specifies the encoder position to be moved to. 
Purpose: Moves motors to specified encoder position. 
Example:
	hand.GoToPosition("1", 10000);
	move F1 to position 10000
*/
```
```c++
void setPositionCommand(const jp_type& jp, unsigned int whichDigits = WHOLE_HAND) const;
```