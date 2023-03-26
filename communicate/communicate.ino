#include <Stream.h>

// String inString[128];
// short readinLen=0;

unsigned char rxBuf[64];	  // 串口接收缓冲
unsigned char *pChar = rxBuf; // 字符指针

// char kp1[10], ki1[10], kd1[10], sv1[10], mode[10], out[10], LR[10];
int serialRxFlag = 0; // 串口接收完标志
char rou1[10], theta1[10];
float rou, theta;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	if (Serial.available())
	{
		Serial.print("jjj\n");
		int inByte = Serial.read(); // 串口读进来的始终是一个整数，即ASCII码值80（助手以ASCII码发送，如'P'）
		// Serial.println((inByte));
		if (('0' <= inByte and '9' >= inByte) or ('A' <= inByte or inByte <= 'Z') or inByte == ',' or inByte == '\n')
		{
			*pChar++ = inByte; // 保存一个字符
			if (rxBuf[0] != 'I')
			{
				memset(rxBuf, '\0', sizeof(rxBuf)); // 数组清零，不清的话串口会有问
				pChar = &rxBuf[0];					// 指针回指，准备下次接收题
			}

			if (inByte == '\n')
			{ // 检查是不是最后一个字符：回车换行符0x0D 0x0A
				serialRxFlag = 1;
			}
			Serial.println((inByte));
		}
	}
	if (serialRxFlag == 1)
	{ // 处理接收的字符串
		serialRxFlag = 0;
		pChar = &rxBuf[0];				   // 指针回指，准备下次接收
		Serial.print("串口发送的命令串:"); // 调试时可以取消注释
		Serial.write(rxBuf, 50);
		// Serial.println();
		// 串口发送例子N,P6.0,I2.5,D0.6,S45.5,M1,T50,R1    第一个字母区别是内回路参数还是外回路参数，注意要发送新行
		// 串口发送例子W,P6.0,I2.5,D0.6,S45.5,M1,T50       第一个字母区别是内回路参数还是外回路参数，注意要发送新行
		if (rxBuf[0] == 'I')
		{ // 解析外回路命令
			// Serial.println("wok");   //调试时可以取消注释
			sscanf(rxBuf, "I,R%[^','],T[^'\n']\n", rou1, theta1);

			// 1.%前面的字符为跳过的字符，如果要跳过多个字符，应全部放在%之前；
			// 2.^为读取的字符串，后面所跟字符为截至字符，就是到逗号为止且丢掉逗号；
			// 3.^须用[]括起，所以一定要核对[]符号的数量
			theta = atof(theta1); // 字符串转成浮点数
			rou = atof(rou1);

			Serial.print("rou");
			Serial.print(rou);
			Serial.print("theta");
			Serial.print(theta);
			Serial.print("\n");

			memset(rxBuf, '\0', sizeof(rxBuf)); // 数组清零，不清的话串口会有问题
		}
	}

}

// ardunio peinrln 为\r\n 