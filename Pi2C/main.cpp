#include "pi2c.h"       
#include <unistd.h>
#include <string>
#include <chrono>
#include <ctime>    


// g++ main.cpp -o main pi2c.cpp
// sudo modprobe i2c-dev

int main(){
	auto time = std::chrono::system_clock::now();
	auto end = std::chrono::system_clock::now();

	Pi2c* interface = new Pi2c(0x15);
	
	char tmp[16];
	while(1){
		int ret = interface->i2cRead(tmp, 16);
		tmp[10] = '\0';
		//if(ret != -1 && tmp[0] != 0x0){
			auto end = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end-time;
			std::cout << ret << " " << (int)tmp[0] << " " << tmp << " " << elapsed_seconds.count() << std::endl;
		//}
		//usleep(100000);
		usleep(1000);
	}
	sleep(5);
	
	
	sleep(1);
	
	char chars1[10] = "abcdefgh\0";
	while(1){
		char chars2[10] = "RV,3.3\0";
	
		interface->i2cWrite(chars2, 10);
		std::cout << "send" << std::endl;
		sleep(1);
		char chars3[10] = "PS,-1\0";
		
		interface->i2cWrite(chars3, 10);
		std::cout << "send" << std::endl;
		sleep(1);
		char chars4[10] = "PS,1\0";
		
		interface->i2cWrite(chars4, 10);
		std::cout << "send" << std::endl;
		sleep(1);
		char chars5[10] = "SK,3\0";
		
		interface->i2cWrite(chars5, 10);
		std::cout << "send" << std::endl;
		sleep(1);
		char chars6[10] = "IR,2,1\0";
		
		interface->i2cWrite(chars6, 10);
		std::cout << "send" << std::endl;
		sleep(1);
		char chars7[10] = "IR,2,0\0";
		
		interface->i2cWrite(chars7, 10);
		std::cout << "send" << std::endl;
		sleep(1);
	}
	
	return 0;
	
	for(int i = 0; i < 4; i++){
		//interface->i2cWriteArduinoInt(255);
		char tmp[3];
		
		int ret = interface->i2cRead(tmp, 3);
		
		//if(ret != -1 && tmp[0] != 0){
			
			std::cout << ret << " " << int(tmp[0]) << (int)tmp[1] << (int)tmp[2] << std::endl;
		//}
		usleep(100000);
		sleep(5);
	}
	
	//interface->i2cWrite(chars5, 10);
	//std::cout << "send" << std::endl;
	//sleep(1);
	//
	//interface->i2cWrite(chars6, 10);
	//std::cout << "send" << std::endl;
	//sleep(1);
	
	return 0;
	
}