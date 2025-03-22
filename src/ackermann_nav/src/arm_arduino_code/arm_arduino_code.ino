/*#include <Servo.h>                           // 引用 ESP8266 伺服舵机 功能库文件
  Servo servo;                                 // 创建舵机对象
  servo.attach(数字引脚,[1000],[2000]);         // 连舵机黄线的引脚,脉冲信号一般设为 500微秒 到 2500微秒
  servo.attached();                            // <bool>获取舵机对象是否已经连接引脚
  servo.detach();                              // 断开引脚控制,使对象不再控制任何舵机

  servo.write(新角度值);                        // 控制转动舵机,数值<200视为角度,否则为脉冲时间(微秒)
  servo.read();                                // <int>获取最后一次write()命令的角度值,0-180 之间
  servo.writeMicroseconds(新脉冲值);            // 控制转动舵机,发送信号脉冲宽度值 500-2500 微秒之间
  servo.readMicroseconds();                    // <int>获取最后一次writeMicroseconds()命令脉冲值

  ESP8266开发板数字引脚 D0-D8 总共9个引脚可用来控制舵机,其中有个引脚与板载LDE灯共享请慎用..
  库文件Servo.cpp里值<200才视为角度,对270度的舵机建议用writeMicroseconds().注意:360度舵机不能控制角度的
  
  9克舵机从一个角度转到另一个角度需要时间,约120毫秒60度.从0度转到180度需要360毫秒,1秒=1000毫秒=100000微秒
  调用write() 5微秒就能返回,同时write()多个舵机,就能看到多舵机同时转动的效果.
  当我们希望舵机是一个接一个轮流转动,或者转到目标角度后再其他操作,可在调用write()后可以再适当的延时 delay();
*/
/*
   控制舵机的高电平脉冲信号在周期20000微秒里占用500-2500微秒时间控制舵机转到指定角度..模拟舵机要持续发信号直到目标角度..
    500微秒----------0度；   1秒=1000毫秒，1毫秒=1000微秒
   1000微秒---------45度；
   1500微秒---------90度；
   2000微秒--------135度；  
   2500微秒--------180度；
   高电平脉冲每 11.11微秒≈1度≈(2500-500)/180,所以1微秒≈0.1度  不用太较真..

   SG90,TS90A 空载转速:0.12秒/60度(4.8V)=120毫秒60度=2毫秒1度
   SG90 工作电压4.8V~6V  待机5ma,空载60ma,工作电流>300ma.最大负载电流700ma.  堵转扭矩:1.2~1.4公斤/厘米(4.8V)
   TS90A工作电压4.8V~6V  待机5ma,空载60ma,工作电流>300ma.最大负载电流700ma.  堵转扭矩:1.6~1.8公斤/厘米(4.8V)
   MG996R 
   注意:电脑蓝色USB3.0插口输出5V900ma电流，，黑色USB2.0输出5V500ma电流(电流严重不够)....
   插电脑USB3.0调试六轴机械臂勉强能用.可外接 5V2a+手机电源 或其他5V8a电源

   舵机三根电线  黄线->接信号(D0~D8),红线->接正极(5V),褐线->接负极(G)

   创客与编程      https://shop104415339.taobao.com/
   四轴机械臂      https://item.taobao.com/item.htm?ft=t&id=634846120104
   板子与舵机      https://item.taobao.com/item.htm?ft=t&id=665636638155
*/
//编译此代码需要先安装ESP8266开发板文件包,并且只能上传到ESP8266芯片的开发板运行.
//arduino 菜单->文件->首选项 添加开发板管理器网址 https://arduino.esp8266.com/stable/package_esp8266com_index.json 
//再 菜单->开发板->开发板管理器 搜esp8266下载最新版本(3.0.0版起)
//注意:ESP8266不太好下载，易出错中断,要多试，换时间试，若还下载不来，可参考网盘里 arduino15 文件夹里的说明...
//请正确选择 菜单->工具->开发板 与 端口COM(需安装CH341串口驱动),8266有好几种板子.选错会导致 pin[6] 变量与板上插口不对应

//arduino 菜单->项目->加载库->管理库 搜 ArduinoJson 安装.
#include <ArduinoJson.h>                   //使用Json文件格式做配置文件
#include "FS.h"                            //ESP8266开发板自带4MB闪存空间,可以用来读写存删文件
#include <Servo.h>                         //引用 舵机 功能库头文件
Servo    S[6];                             //创建舵机对象
Servo    O, P;                             //O 电子阀 插D6;P 真空泵 插D7;

int        ss = 4;                             //4=4轴,5=5轴,其他值或6=6轴6舵机械臂.此变量值决定使用下面数组几个成员值
char  XYZE[6] = {'X','Y','Z','E','B','T'};     //定义6个电机从底座到夹子为 XYZBTE.5轴没有T舵机
int    pin[6] = { D2, D3, D0, D8, D5, D6};     //开发板的数字针脚用来接6个电机.5轴没有T舵机
int rawdms[6] = { 90, 90, 90, 90, 90, 90};     //原点,起点,通电后或H指令会让所有舵机归位到此脉宽.500=0度，1500=居中90度，2500=180度
int olddms[6] = { 90, 90, 90, 90, 90, 90};     //每次转动舵机后保存脉宽值到此,作为下次转动的起点.
int newdms[6] = { 90, 90, 90, 90, 90, 90};     //每次转动舵机后保存脉宽值到此,作为下次转动的起点.
int mindms[6] = {  0, 50, 90, 10,  0,  0};     //定义6个电机在机械臂中可转动的最小信号脉冲微秒值
int maxdms[6] = {180,180,180,100,180,180};     //定义6个电机在机械臂中可转动的最大信号脉冲微秒值

bool         Step = true;                    //true=减速(1角度1角度的转),false=0=舵机原速转动
float       factor=11.11;                    //11.11=(2500-500)/180度
volatile int Autorun = 0;                    //自动运行动作指令开关  0=不运行,>0为循环运行次数
String     Cmd,Cmdret="";                    //把一些指令放在这个变量,下次loop循环时执行

//---------------把 0-180 角度值转为对应高电平脉冲信号时间宽度 ---------------
int todms(float V) {                        //返回  把角度值转换为脉冲宽度值
      if(V<360.0){                          //值少于360视为角度,否则视为脉宽
        V=V*factor;    
        if((float)(V-(int)V)>=0.45) V=V+1.0;
        return 500+(int)V;                  //180度舵机取值0~180间并转为脉宽
      } else {
        return constrain((int)V,500,2500);  //取值500~2500间,9克舵机都是这个
      }
}

//----------------------------------------------------------------------
//舵机对信号是单向处理的,可以发命令给舵机要求转到某个角度.
//但舵机有没有转到那个位置和目前的状态是不会返回给我们的..
//所以需要自已用变量保存上次的角度与现在要转到的角度..
//--------------函数 Servo180( )----调用Servo.h里的功能控制舵机转动-------------
unsigned long moveServos() {
    unsigned long startTime = millis();  // 记录开始时间
    int steps[6];  // 假设最多6个舵机
    int maxSteps = 0;
    
    // 计算每个舵机需要的步数
    for(int i = 0; i < ss; i++) {
        steps[i] = abs(newdms[i] - olddms[i]);
        maxSteps = max(maxSteps, steps[i]);
    }
    
    // 如果不需要移动，直接返回
    if(maxSteps == 0) return 0;
    
    // 计算预期总耗时
    // 空载速度：0.12秒/60度 = 2毫秒/度
    unsigned long expectedTime = startTime + (maxSteps/(int)factor * 5);
    
    // 同步移动所有舵机
    if(Step) {
        // 减速模式
        for(int step = 0; step < maxSteps; step += factor) {
            for(int i = 0; i < ss; i++) {
                if(olddms[i] != newdms[i]) {
                    int diff = newdms[i] - olddms[i];
                    int move = constrain(diff, -factor, factor);
                    olddms[i] += move;
                    S[i].write(olddms[i]);
                }
            }
            delay(1);  // 基本延时
            yield();   // 防止触发看门狗重启
        }
    } else {
        // 直接移动模式
        for(int i = 0; i < ss; i++) {
            if(olddms[i] != newdms[i]) {
                S[i].write(newdms[i]);
                olddms[i] = newdms[i];
            }
        }
    }
    
    // 确保运动时间符合预期
    unsigned long currentTime = millis();
    if(expectedTime > currentTime) {
        // 限制最大延时为1秒
        unsigned long delayTime = constrain(expectedTime - currentTime, 0, 1000);
        delay(delayTime);
    }

    return expectedTime;
}
bool Servo180(int I,int Value) {   //脉宽高电平 500微秒 到 2500微秒 之间，对应舵机0°～180°可转角度
  int count=0;                            //对应的角度值

  //------------------------------------------------------------------------
  if(I>=0 && I<ss){                       
    if(newdms[I]!=olddms[I]){             //检查上次设置的该舵机新角度是否已执行
      Servo180(-1,0);                     //若未执行,则立即执行
    }
    newdms[I]=constrain(todms(Value),mindms[I],maxdms[I]); //转为脉宽数值存入

    if(ss==4) Servo4(I); //四轴机械臂 限制Y轴与Z轴最大钝角与最小锐角,与装配角度有关
    return false;
  }

  unsigned long maxms = moveServos();

  return maxms!=0;     //有发送过舵机信号返回 true
}

void Servo4(int I) {
    const int MIN_ANGLE_SUM = 3350;
    const int MAX_ANGLE_SUM = 4550;
    
    struct Vec2 {
        int& y;
        int& z;
        Vec2(int& y_, int& z_) : y(y_), z(z_) {}
        
        int sum() const { return y + z; }
        
        void adjust(int target_sum) {
            int current_sum = sum();
            int diff = target_sum - current_sum;
            // 按比例分配调整量
            float ratio = 0.5f;  // 可以根据需要调整分配比例
            y += diff * ratio;
            z += diff * (1 - ratio);
        }
    };
    
    Vec2 angles(newdms[1], newdms[2]);
    int angle_sum = angles.sum();
    
    // 调整总角度到允许范围
    if(angle_sum < MIN_ANGLE_SUM) {
        angles.adjust(MIN_ANGLE_SUM);
    } else if(angle_sum > MAX_ANGLE_SUM) {
        angles.adjust(MAX_ANGLE_SUM);
    }
    
    // 确保在各自范围内
    newdms[1] = constrain(newdms[1], mindms[1], maxdms[1]);
    newdms[2] = constrain(newdms[2], mindms[2], maxdms[2]);
}
//
//------------------载入配置文件，修改一些变量数据参数------------------------
void loadConfig() {                            //载入配置文件/config.json替换掉一些变量中的值
    File F = SPIFFS.open("/config.json", "r"); //打开读取配置文件
    F.seek(0);                                 //到首位置
    String S=F.readString();                   //读入文本
    F.close();                                 //关闭文件

    if (S.length()>32) {
       //StaticJsonDocument<1024> doc;          //栈内存JSON文档对象
       DynamicJsonDocument doc(2048);           //堆内存JSON文档对象
       deserializeJson(doc,S);                  //把内容装载到JSON对象
       //serializeJson(doc, Serial);              //输出JSON格式内容到串口
       //Serial.println();                        //输出换行符
              
       ss=doc["ss"];                            //ss 4=4轴机械臂,5=5轴机械臂,其他值或6=六轴机械臂

       for(int I=0;I<ss;I++){                   //读取舵机参数到数组变量
          const char* C    =   doc["Servo"][I]; //舵机编号 
          XYZE[I]=*C;
          pin[I]     = doc["pin"][I];           //舵机GPIO
          rawdms[I]  = todms(float(doc["rawdms"][I])); //原脉宽 1500=居中90度
          mindms[I]  = todms(float(doc["mindms"][I])); //最小脉宽微秒值
          maxdms[I]  = todms(float(doc["maxdms"][I])); //最大脉宽微秒值
       }
       Autorun = doc["Autorun"];                //板子通电自动运行Auto.txt次数
       doc.clear();
    }
}
//-------------------------------------------------------------
String output(){                //返回Json格式的所有舵机当前角度信息
  StaticJsonDocument<256> doc;              //栈内存JSON文档对象

  for(int I=0;I<ss;I++){
     String s=String(XYZE[I]);                       //舵机编号
     float  v=(float)(newdms[I]-500.0)/factor;
     doc[s]=String(v,1); //输出带1位精度的角度值
  }
  String ret;
  serializeJson(doc,ret);                   //单行格式 到变量
  //serializeJsonPretty(doc,ret);           //多行格式 到变量
  doc.clear();
  return  ret;   //{"X":90,"Y":90,"Z":90,"E":90,"B":90,"T":90}
}
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
ICACHE_RAM_ATTR void ISR(){Autorun=1;}               //D1中断事件处理
//-----------------------------------------------------------------------
//------------------开发板通电后初始函数 setup()----------------------------
void setup() {
    Serial.begin(115200);                 //开启串口通信 波特率115200,串口监视器也要相同波特率,不然会乱码
    pinMode(LED_BUILTIN , OUTPUT);        //初始化8266开发板LED信号灯的GPIO口为输出.
    digitalWrite(LED_BUILTIN,LOW);        //Mini ESP8266板LED_BUILTIN=GPIO 2,LOW=亮灯,HIGH=灭灯
    //for(int i=0;i<1000;i++) delay(2);
    Serial.printf("\nsetup  LED_BUILTIN:%d\n",LED_BUILTIN);//输出LED信号灯用的 芯片GPIO编号
    //注意:Mini D1 ESP8266开发板LED_BUILTIN=GPIO2=D4,为避免冲突,舵机信号线不要插到D4去,上面数组里也没用D4.

    //------------ arduino 菜单->工具->Flash Size->4MB(FS:2MB OTA:1019KB) -----------------
    //ESP8266开发板代码.格式化并建立内置闪存文件系统,用来保存 Auto.txt 等机械臂自动化动作指令
    //SPIFFS.format();         //第一次使用 格式化SPIFFS,可清除所有内容
    SPIFFS.begin();            //开启闪存文件系统
    FSInfo info;               //信息
    SPIFFS.info(info);         //取闪存文件系统信息

    Serial.printf("\n闪存.已用:%d字节;可用:%d字节;",info.usedBytes,info.totalBytes);         //已用空间,可用空间
    //Serial.printf("块大小:%d  页大小:%d",info.blockSize,info.pageSize);                   //存储块大小,存储页大小
    //Serial.printf("最长文件名:%d  最多可打开:%d\n",info.maxPathLength,info.maxOpenFiles);  //最长文件名,最多可打开文件

    loadConfig();              //载入配置文件(如果有),替换一些变量值
    //-------------设置舵机数字插口与脉冲宽度微秒时间,驱动舵机到初始角度---------------
    if(ss!=4 && ss!=5) ss=6;                                  //ss 决定程序控制是几轴机械臂
    if(ss==4 && todms((float)rawdms[2])<2000) rawdms[2]=2490; //四轴Z舵机默认在 180度 位置
    for(int I=0; I<ss; I++){
      S[I].attach(pin[I],500,2500);            //绑定针脚,设置信号脉冲宽度范围//S[I].detach();
      rawdms[I]=todms((float)rawdms[I]);
      olddms[I]=rawdms[I];                     //rawdms变量拷贝给olddms变量
      newdms[I]=rawdms[I];                     //rawdms变量拷贝给newdms变量
      mindms[I]=todms((float)mindms[I]);
      maxdms[I]=todms((float)maxdms[I]);

      S[I].write(rawdms[I]);                   //写入新角度值,控制舵机转动
      delay(500);                              //等待该舵机转到目标角度.
    }
    O.attach(D6,500,2500);O.write(0);  // 创建电磁阀对象 与 D6针脚关联  默认为断电关阀，与 插管子有关 5V约200ma~300ma 
    P.attach(D7,500,2500);P.write(0);  // 创建真空泵对象 与 D7针脚关联  默认为关泵状态，370微型真空泵 5V约450ma~600ma 

          
    Serial.printf("\n初始化舵机状态:数量%d;",ss);

    pinMode(D1,OUTPUT);                     //设置D1为输出模式
    //---使用D1引脚为脚本启动开关,当D1与 5V正极接触,中断调用 ISR 设置 Autorun=1 运行 Auto.txt 动作脚本文件-----
    digitalWrite(D1,LOW);                   //设置D1为低电平 LOW状态
    attachInterrupt(digitalPinToInterrupt(D1),ISR,RISING);//启用硬件中断实时捕获 D1 引脚低电平LOW变为高电平HIGH
    //---使用D1引脚为脚本启动开关,当D1与GND负极接触,中断调用 ISR 设置 Autorun=1 运行 Auto.txt 动作脚本文件-----
    //digitalWrite(D1,HIGH);                //设置D1为高电平HIGH状态
    //attachInterrupt(digitalPinToInterrupt(D1),ISR,FALLING);//启用硬件中断实时捕获 D1 引脚高电平HIGH变为低电平LOW
    //CHANGE（改变沿，电平从低到高或者从高到低）、RISING（上升沿，电平从低到高）、FALLING（下降沿，电平从高到低）


    digitalWrite(LED_BUILTIN,HIGH);          //亮灯
    delay(500);                              //等待该舵机转到目标角度.
    digitalWrite(LED_BUILTIN,LOW);          //亮灯
    delay(500);                              //等待该舵机转到目标角度.
    digitalWrite(LED_BUILTIN,HIGH);          //亮灯
    delay(500);                              //等待该舵机转到目标角度.
    digitalWrite(LED_BUILTIN,LOW);          //亮灯
    delay(500);                              //等待该舵机转到目标角度.
    digitalWrite(LED_BUILTIN,HIGH);          //亮灯
    delay(500);                              //等待该舵机转到目标角度.
    digitalWrite(LED_BUILTIN,LOW);          //亮灯

    Serial.printf("\n初始化系统完成");
}
//-----------------------------------------------------------------
//
//
//
//------------------开发板循环调用函数 loop()------------------------
void loop() {

    // if(Autorun!=0){                           //开启循环执行Auto文件命令
    //   Command("R Auto.txt");                  //执行 R 命令
    //   if(Autorun>0) Autorun--;                //正数减1次,负数不变
    // }
    // if(Cmd!=""){                              //如果cmd全局变量里有未执行命令,现在执行
    //   Cmdret=split(Cmd);                      //分析指令并调用Command执行
    //   Cmd="";
    // }
    
   //---------------监听电脑串口输入指令-------------------
   if(Serial.available()>0){                   //有串口数据>0字节
    String line = Serial.readStringUntil('\n');
    // String t=Serial.readString();             //?SDCHRA XYZBTE
    line.trim();                                 //删首尾空格与换行
   //t.toUpperCase();                          //指令转为大写
    Serial.printf("串口输入指令:%s\n",line);      //串口回显指令
    Autorun = 0;                              //有命令时停止动作
    Serial.println(split(line));
   }
    yield();  // 防止看门狗复位
}

//---------------------------------------------------
// 舵机控制命令支持 单个执行也支持多个同时执行
// 单命令   X50
// 多命令   X50;Y100;Z100
// 多行命令 X50;Y100\nZ100;E80
//---------------------------------------------------
String split(String t) {                    //拆分多行命令 或以 ; 为分隔的多个命令
     String ret="";
     long i[2]={0};                         //[0]=; [1]=\n
     do{
       i[0]=t.indexOf(";" ,0);              //查找分隔符 ;
       i[1]=t.indexOf("\n",0);              //查找换行符 \n
       if(i[0]==-1 && i[1]==-1){            //没找到符号
         ret=Command(t); break;             //单个指令执行跳出
       } else if(i[0]!=0 && i[1]==-1){      //单行有 ; 分隔的多个指令
         ret=Command(t.substring(0,i[0]));  //提取一个指令执行
         t.remove(0,i[0]+1);                //删除已执行的指令
       } else if(i[1]!=0 && i[0]==-1){      //多行的 无分隔的单个指令
         ret=Command(t.substring(0,i[1]));  //提取单行指令执行
         t.remove(0,i[1]+1);                //删除已执行的此行
         if(Servo180(-1,0))Serial.println(output());
       } else if(i[0]<i[1]) {               // ; 在 \n 之前
         ret=Command(t.substring(0,i[0]));  //提取一个指令执行
         t.remove(0,i[0]+1);                //删除已执行指令
       } else if(i[0]>i[1]){                // \n 在 ; 之前
         ret=Command(t.substring(0,i[1]));  //提取一行指令执行
         t.remove(0,i[1]+1);                //删除已执行指令
         if(Servo180(-1,0))Serial.println(output());
       } else break;
     }while(t.length()>0);                  //未查找完 继续循环
     if(ret=="") if(Servo180(-1,0)) ret=output();
  return ret;
}

//-----------------解析并执行命令 已定义常用命令 ?SDCHRA XYZBE----------------
String Command(String t)  {
    static int  dms[6]={0};                //静态变量,存放上次S保存过的电机位置 
    static String file="/Auto.txt";        //静态变量,存放FSDRC命令将操作的文件
    
    int    i=0,j=0,v=0;                    //定位指令文本中位置
    String   Auto;                         //Auto.txt文件内容
    String   S="";                         //保存命令所带的参数
    String ret="";                         //返回结果

    //-------------------拆分以空格分隔的命令与参数------------------
    t.trim();                              //删首尾空格与换行
    i=t.indexOf(" ",0);                    //查找空格 拆分命令与参数
    if(i>0){
      S=t.substring(i+1);                  //提取参数
      S.trim();                            //删首尾空格与换行
      t.remove(i);                         //保留命令,删除参数
      //Serial.println("Command.命令:s% 参数:s%\n",t,S);//串口输出拆分出来的命令和参数
    }

    //----------------解析执行一些命令 ?SDCHRA --------------------------------
    if(t.equalsIgnoreCase("RE")){          //re 重启开发板
        ESP.restart();                     //软重启
        return "";
    }
    if(t=="?" || t=="？"){                  //？HELP 输出简要的帮助信息
        if(S=="") S="/HELP.txt";            //? Auto 输出指定文件的内容
        if(!S.startsWith("/"))  S="/"+S;    //比较字符串前缀
        if(1>S.indexOf(".",1))  S=S+".txt"; //比较字符串后缀
        File F=SPIFFS.open(S,"r");          //"r"=只读内容,"w"=重写内容
        F.seek(0);                          //到首位置
        ret=F.readString();                 //读入文本
        F.close();                          //关闭文件
        return ret;
    }
    if(t.startsWith("//")){                 //跳过动作文件中的备注行
        return "";
    }
    if(t.equalsIgnoreCase("F")){            //用 "F Auto.txt" 改变动作文件名
        if(S=="") S="/Auto.txt";            //用 "F test.txt" 指定动作文件名
        if(!S.startsWith("/"))  S="/"+S;    //比较字符串前缀
        if(1>S.indexOf(".",1))  S=S+".txt"; //比较字符串后缀
        file=S;                             //保存文件名 R S D C 将对该文件操作
        return ("{\"F\":\""+file+"\"}");    //输出新动作文件名称
    }
    if(t.equalsIgnoreCase("delay")){        //动作脚本中 delay xx 由这段代码执行
        int v=S.toInt();                    //delay的参数,即是要延时的毫秒时间
        unsigned long ms=millis();          //板子开机已经过的毫秒时间
        Servo180(-1,0);
        while(millis()-ms<v){               //在要延时的毫秒时间里一直循环
            delay(0);
        }
        return "";
    }
    if(t.equalsIgnoreCase("SH")){               //保存XYZBTE舵机当前位置到 H.txt 文件
        File F=SPIFFS.open("/H.txt","w");       //"w"=重写文件所有内容
        F.seek(0);                              //到首位置
        for(int I=0;I<ss;I++){                  //循环所有舵机变量进行输出保存
           float f=(float)(newdms[I]-500)/factor;
           F.printf("%c%.1f\n",XYZE[I],f);      //保存新位置到文件
        }
        F.close();                              //关闭文件
        return Command("? /H.txt");
    }
    if(t.equalsIgnoreCase("H")){             //HOME 让XYZBTE六个电机到指定位置
        if(SPIFFS.exists("/H.txt")){         //如果有 H.txt 则执行
           Command("R H.txt");
           return output();
        } else for(int I=0;I<ss;I++){        //无 H.txt 则执行rawdms变量值
           Servo180(I,rawdms[I]);
        } 
        Servo180(-1,0);
        return output();
    }
    if(t.equalsIgnoreCase("Test")){          //执行一段底座左右转动的测试动作
        ret=Test();
        return ret;
    }
    if(t.equalsIgnoreCase("S")){                       //保存XYZETB电机新角度指令到自动执行文件
        ret="";
        if(S.toInt()>0){                               //S xx 转为 delay xx 保存到文件
            ret="delay "+String(S.toInt());            //保存delay xx到文件
        } else if(S!=""){                              //S delay xx 保存 delay xx  到文件
            ret=S;                                     //S Y1500;S R test.txt;直接存到文件
        } else for(int I=0;I<ss;I++){                  //判断位置发生变化的所有舵机并保存
            if(dms[I]!=newdms[I]){                     //判断位置与上次保存后有无发生变化
              //dms[I]=map(newdms[I],500,2500,0,180);  //转为角度值输出
              if(ret!="") ret+=";";                    //如果有多个舵机有转动过,添加 ; 分隔符
              float f=(float)(newdms[I]-500)/factor;
              ret+=XYZE[I]+String(f,1);                //保存新位置到文件   
              dms[I]=newdms[I];                        //保存新的X位置下次判断用
            }
        }
        if(ret!=""){                            //有 待追加命令
            File F=SPIFFS.open(file,"r");       //"r"=读取内容
            F.seek(0);                          //到首位置
            Auto=F.readString();                //读入文本
            Auto.trim();                        //删首尾空格与换行
            F.close();                          //关闭文件

            F=SPIFFS.open(file,"w");            //"w"=重写文件所有内容
            F.println(Auto);                    //先保存原指令
            F.println(ret);                     //再追加新指令
            F.flush();F.close();                //结束关闭文件
        }
        return ret;                             //返回信息串口输出新指令
    }
    if(t.equalsIgnoreCase("D")){            //D 1 删除最后一行;  D 3 删除多行动作指令
        File F=SPIFFS.open(file,"r");       //"r"=只读内容,"w"=重写内容
        F.seek(0);                          //到首位置
        Auto=F.readString();                //读入文本
        Auto.trim();                        //删首尾空格与换行
        F.close();                          //关闭文件

        v=S.toInt();                        //D 3 删除末尾3行
        if(v<1)v=1;
        for(;v>0;v--){                      //循环次数是要删除几行
          i=Auto.lastIndexOf("\n");         //倒找换行符
          if(i<0) i=0;                      //-1=无换行符(改0,删除所有内容)
          Auto.remove(i);                   //删除i后面的文本
        }

        F=SPIFFS.open(file,"w");            //"w"=重写文件所有内容
        F.seek(0);                          //到首位置
        F.print(Auto);                      //保存剩余指令到文件
        F.close();                          //关闭文件
        return Auto;                        //输出删行后的剩余 动作指令
    }
    if(t.equalsIgnoreCase("C")){            //C Auto;C test;清空指定文件内容并删除文件
        if(S=="") S=file;                   //参数若未指令文件，删除file
        if(!S.startsWith("/"))  S="/"+S;    //比较字符串前缀
        if(1>S.indexOf(".",1))  S=S+".txt"; //比较字符串后缀
        SPIFFS.remove(S);                   //删除文件
        dms[6]={0};
        return S;                           //输出删除的文件
    }
    if(t.equalsIgnoreCase("step")){         //改变舵机速度,step 0=原速,step 1=减速;                   
        Step = 0!=S.toInt();              
        return "Step:"+String(Step);        //
    }
    if(t.equalsIgnoreCase("format")){   //格式化 开发板闪存文件系统 清除所有文件,需要重新上传
        SPIFFS.format();                  //格式化 
        FSInfo info;                      //信息
        SPIFFS.info(info);                //取闪存文件系统信息
        ret="闪存.已用:"+String(info.usedBytes)+"字节;可用:"+String(info.totalBytes)+"字节;";
        return ret;
    }
    if(t.equalsIgnoreCase("UP")){   //UP and down  你想写啥就写啥,别问我
        Servo180(-1,0); 
        ret="UP..";
        return ret;
    }
    if(t.equalsIgnoreCase("DIR")){       //枚举文件并输出
        FSInfo info;                     //信息
        SPIFFS.info(info);               //取闪存文件系统信息
        ret="闪存.已用:"+String(info.usedBytes)+"字节;可用:"+String(info.totalBytes)+"字节;\n";

        Dir dir=SPIFFS.openDir("/");     //打开根目录,枚举文件名
        while(dir.next()) {              //循环所有对象
           ret+=dir.fileName();
           File F=dir.openFile("r");
           ret+="\t";                    //Tab 制表符
           ret+=String(F.size());        //取文件长度
           ret+="\n";
           F.close();
        }
        return ret;                      //返回输出所有枚举到的文件信息
    }

    if(t.equalsIgnoreCase("O")){             // 电磁阀控制 //电磁阀  铁 #1 IN ，直 #2 OUT,侧 #3 OUT //V 0 断电 2死，1-3相通，，V 180 通电 1死，2-3相通
        if(S.toInt()<0){                     // O -1 ,上电 进气 卸物
          P.write(0);                        // 电磁阀进气之前先关泵
          O.write(180);                      // 电磁阀 上电
        }else if(S.toInt()>0){               // O 500 上电 进气 卸物,可以继续抽真空
          P.write(0);                        // 电磁阀进气之前先关泵
          O.write(180);                      // 电磁阀 上电 进气 卸物
          if(S.toInt()<500) S="500";         // 最小进气时间为 0.5秒
          if(S.toInt()>10000) S="10000";     // 最大进气时间为  10秒
          Command("delay "+S);               // 进气 卸物 时间控制
          //delay(S.toInt());
          O.write(0);                        // 时间到 断电 通阀,可以继续抽真空,与气管插管有关
        } else {O.write(0);delay(100);}      // O 0 断电 通阀,可以继续抽真空,与气管插管有关
        return String(S);
    }

    if(t.equalsIgnoreCase("P")){             //真空泵控制;关泵=P 0,开泵=P -1,开泵1秒=P 1000 ;推荐1秒到3秒就能吸住物体
        if(S.toInt()>30000) S="30000";       //建议每次开泵时间不超过30秒，休息10秒
        if(S.toInt()!=0){                    //开泵
          O.write(0);                        //断电 通阀，与气管插管有关
          P.write(180);                      //开泵
          if(S.toInt()>0){                   //非 P -1
            Command("delay "+S);             //开泵时间控制
            //delay(S.toInt(delay));         //
            P.write(0);                      //延时结束关泵
          }                                  //P -1 
        } else P.write(0);                   //关泵=P 0
        return String(S);
    }

    if(t.equalsIgnoreCase("R")){        //R   运行file变量指定文件的动作指令
        v=S.toInt();                    //R 3 执行 Auto.txt 3次
        if(v!=0 && S==String(v)){        //R Auto   执行Auto.txt 1次
           Autorun=v;                   //R test   执行test.txt 1次
           return ret;                  //若 R 参数为整数,设置 Autorun 变量循环执行 Auto.txt 次数
        }
        if(S=="") S=file;               //参数若未指定文件,则执行file变量里的文件
        Auto=S;
    } else Auto=t;                      //对于不可识别的指令尝试以动作文件执行,如 "Q" 变会成 "/Q.txt" 若存在则执行 
    if(!Auto.startsWith("/"))  Auto="/"+Auto;    //比较字符串前缀
    if(1>Auto.indexOf(".",1))  Auto=Auto+".txt"; //比较字符串后缀
    if(SPIFFS.exists(Auto)){  //判断动作文件是否存在.若有此文件则执行
        ret="R "+Auto+"\n";
        v=S.toInt();                        //Auto.txt 5 提取执行次数
        if(v<1 || v>256 || S!=String(v))    //
           v=1;                             //不符合要求的次数重定为1次
        S=Auto;                             //另存文件名称
        for(;v>0;v--){
           File F=SPIFFS.open(S,"r");       //打开文件 "r"=只读取文件内容  
           //F.seek(0);                     //到文件首位置
           Auto=F.readString();             //读内容到变量
           F.close();                       //关闭文件

           //拆分多行命令 或以 ; 为分隔的多个命令
           int Run=Autorun;               //要运行的次数,当值有改变时停止
           i=0;j=0;                                 //i起始,j=;,v=\n
           do{
              i=Auto.indexOf(";" ,0);               //查找分隔符 ;
              j=Auto.indexOf("\n",0);               //查找换行符 \n
              if(i==-1 && j==-1){                   //没找到符号
                 Command(Auto); break;              //单个指令执行跳出
              } else if(i!=0  && j==-1){            //单行有 ; 分隔的多个指令
                 Command(Auto.substring(0,i));      //提取一个指令执行
                 Auto.remove(0,i+1);                //删除已执行的指令
              } else if(i==-1 && j!=0){             //多行的 无分隔的单个指令
                 Command(Auto.substring(0,j));      //提取单行指令执行
                 Auto.remove(0,j+1);                //删除已执行的此行
                 Servo180(-1,0);
              } else if(i<j) {                      // ; 在 \n 之前
                 Command(Auto.substring(0,i));      //提取一个指令执行
                 Auto.remove(0,i+1);                //删除已执行指令
              } else if(i>j){                       // \n 在 ; 之前
                 Command(Auto.substring(0,j));      //提取一行指令执行
                 Auto.remove(0,j+1);                //删除已执行指令
                 Servo180(-1,0);
              } else break;
           }while(Auto.length()>0 && Run==Autorun); //-1=没找到换行 或 运行次数发生变化,结束循环.
           Servo180(-1,0);
        }
        //Serial.println("R end");
        return ret+output();
    }


    //----------解析XYZBTE电机指令  底座X 前后Y 上下Z 平衡B 旋转T 夹子E
    t.toUpperCase();                   //指令转为大写
    for(i=0;i<ss;i++){                 //循环所有舵机
      if(XYZE[i]==t.charAt(0)){        //判断指令第1个字符为哪个电机
        if(t.length()==1){             //X --;  X ++;  X 1500;
          if(S=="--"){
            v=newdms[i]-1;             //新角度=旧角度-1度(约11微秒)
          } else if(S=="++"){
            v=newdms[i]+1;
          } else {
            v=newdms[i]+S.toInt();       //tofloat
          }
          Servo180(i,v);                 //执行电机指令
          //Serial.println(v);           //串口输出执行的指令
          return "";
        }


        if(t.charAt(2)=='-'){          //X--    第3个字符
          v=newdms[i]-(int)factor;     //新角度=旧角度-1度(约11微秒)
        } else if (t.charAt(2)=='+'){  //X++    第3个字符
          v=newdms[i]+(int)factor;     //新角度=旧角度+1度(约11微秒)
        } else if (t.charAt(1)=='-'){  //X-?    第2个字符
          t=t.substring(2);            //提取第3个字符与后面的数字内容
          
          v=newdms[i]-(int)(t.toFloat()*factor);//新角度=旧角度-N度(约11微秒)
        } else if (t.charAt(1)=='+'){  //X+?    第2个字符
          t=t.substring(2);            //提取第3个字符与后面的数字内容
          float f=t.toFloat()*factor;
          if((float)(f-(int)f)>=0.45) f=f+1.0;
          v=newdms[i]+(int)f;          //新角度=旧角度+N度(约11微秒)
        } else {                       //X??? 绝对定位
          t=t.substring(1);            //提取第2个字符与后面的数字内容
          v=todms(t.toFloat());        //新角度=数字内容值 或 新脉宽值
        }
        Servo180(i,v);                 //执行电机指令
        //Serial.println(i); 
        //Serial.println(v);           //串口输出执行的指令
        return "";
      }
    }
    return ret;
}

//---------------------------------------------------------------------------
//收到 test 指令会执行这个测试程序,控制 X 和 E 舵机转动
//可供参考用,自已编程使用 变量存取 与 逻辑条件 做复杂功能
//---------------------------------------------------------------------------
String Test() {         //一个简单的测试底座来回转到的功能

    Command("step 1");                   //设定为 减速
    Command("H");                        //所有舵机复位
    for(int i=1;i<6;i++){
      Command("X"+String(90+i*10));      //X底座转向一侧
      Command("E-20");                   //E开夹20度
      if(i==4) Command("Y+10");          //当左右转第4次时Y前顷10度
      Command("X"+String(90-i*10));      //X底座转向另一侧
      Command("E+20");                   //E闭夹20度
    }
    Command("H");                        //所有舵机复位
    return "Test()";
}

//---------------------------------------------------------------------------------------
/*#include <FS.h>  工具->Flash Size->配置用户闪存大小 
FSInfo info
SPIFFS.begin()     //启动ESP8266闪存文件系统
SPIFFS.info(info)  取闪存文件系统信息
info.totalBytes    可用空间
info.usedBytes     已用空间
info.maxPathLength 最长文件名
info.maxOpenFiles  最多可打开文件
info.blockSize     存储块大小
info.pageSize      存储页大小

SPIFFS.format()    //格式化SPIFFS,清除内容
SPIFFS.exists(文件名)         //文件是否存在
SPIFFS.remove(文件名)         //移除文件
File F=SPIFFS.open(文件,"w")  //"r","w"=重写内容,"a"=追加内容 



File类读取和写入数据到文件。
   File F=SD.open(文件[,FILE_READ|FILE_WRITE])
 * F.available()      //返回可读取字节数
 * F.close()          //关闭文件，并确保写入其中的所有数据都已物理保存到SD卡中。
 * F.flush()          //确保写入文件的所有字节都已物理保存到SD卡。关闭文件后，此操作会自动完成
 * F.peek()           //byte或-1.从文件中读取一个字节，而无需前进到下一个字节
 * F.position()       //获取文件中的当前位置
 * F.print(数据[,格式])//将数据打印到文件中，该文件必须已打开才能进行写入,（char，byte，int，long或string）,BIN（二进制）（基数2），DEC（十进制）（基数10），OCT（八进制）（基数8），HEX（十六进制）
 * F.println(数据[,格式]) //将数字打印为数字序列，每个数字为一个ASCII字符（例如，数字123作为三个字符" 1"，" 2"，" 3"发送）。
 * F.seek(新位置)      //true|false 在文件中寻找新位置，该位置必须介于0和文件的大小（含）之间
 * F.size()           //unsigned long 获取文件的大小
 * F.read()           //byte (or character), or -1 从文件中读取一个字节
 * F.write(数据[,长度])//将数据写入文件。
 * F.isDirectory()          //boolean 目录（或文件夹）是特殊类型的文件，此功能报告当前文件是否为目录
 * F.openNextFile(句柄)      //char 报告目录中的下一个文件或文件夹
 * F.rewindDirectory(句柄)   //将带您回到目录中的第一个文件，与openNextFile（）结合使用
 * 
 * 
ESP32 开发板的文件函数
    F.size()                     //<size_t>获取文件的大小
    F.available()                //<int>返回可读取字节数
    F.seek(新位置[,模式]);         //<bool>0=文件首,1=现位置,2=文件尾
    F.position()                 //<size_t>获取文件中的当前位置
    
    F.peek()                     //<int>拷贝1字节,指针不变
    F.read()                     //<int>读取1字节,-1=失败
    F.read(uint8_t* 数组,长度)    //<size_t>读取数据
    F.readBytes(char *数组,长度)  //<size_t>读取字节集(uint8_t*)
    
    F.write(数据[,长度])          //<size_t>写入数据,返回位置
    F.flush()                   //确保写入文件的所有字节都已物理保存到SD卡。关闭文件后，此操作会自动完成



    F.bool() const;       //<operator>返回对象
    F.name()              //<char*>获取对象名称
    F.getLastWrite();     //<time_t>获取最后写入时间

    F.isDirectory(void)   //<bool>是否为目录
    F.openNextFile(模式)  //<File>打开下个文件[FILE_READ]
    F.rewindDirectory(void);  //将带您回到目录中的第一个文件，与openNextFile（）结合使用
    F.close();                 //立即F.flush()并关闭文件
 */




/* String 文本对象  t.String(数值字符与串,[格式])  
String S = "你好文本" ;                             //使用常量String
String S =  String('a');                           //将常量char转换为String
String S =  String("This is a string");            //将常量文本转换为String对象
String S =  String(stringTwo+"with more");         //连接两个文本
String S =  String(13);                            //使用常量整数
String S =  String(analogRead(0),DEC);             //转为10进制整数文本
String S =  String(45,HEX);                        //转为16进制整数文本
String S =  String(255,BIN);                       //转为 2进制整数文本
String S =  String(millis(),DEC);                  //转为10进制长整数文本
String S =  String(5.698,3);                       //转为3位精度小数文本
String S =  String(char[]);                        //字符串到文本null结尾.

    t.charAt(位置)            <char>取字符
    t.compareTo(子串)         <int>字符串排序比较,-值子串在后,0=相同,+值子串在前
    t.concat(子串)            <String>字符串组合
    t.c_str()                <int>取字符串指针,null终止.不要用指针修改字符串,字符串变动时指针会失效.

    t.equals(子串)            <bool>比较字符串,区分大小写
    t.equalsIgnoreCase(子串)  <bool>比较字符串,不区分大小写
    t.startsWith(字符串)       <bool>比较字符串前缀,区分大小写
    t.endsWith(字符串)         <bool>比较字符串后缀,区分大小写
    
    t.getBytes(byte[],长)     转为字节数组保存
    t.toCharArray(char[],长)  转为字符数组保存
    t.indexOf(字符或串,位置)     <long>查找字符串,返回字符数位置,非内存长度位置,失败-1
    t.lastIndexOf(字符或串,位置) <long>倒找字符串,返回字符数位置,非内存长度位置,失败-1
    t.length()                 <long>字符串字符数,非内存占用长度
    t.remove(起始,[长度])      删除子串
    t.replace([子串],新串)     <String>替换字符串      
    t.reserve(内存长度)        申请内存
    t.setCharAt(位置,char)    修改字符

    t.substring(起始,[截止])   <String>取子字符串
    t.toInt()                <long> 转为整数.失败返回零
    t.toFloat()              <float>转为小数.浮点数的精度只有6-7个十进制数字
    t.toLowerCase()          转为小写
    t.toUpperCase()          转为大写
    t.trim()                 删首尾空格与换行

char[] 字符串数组,通常,字符串以空字符(ASCII代码0)结尾,数组应该多1字节
  char Str1[9];                                              //声明一个char数组，而不像在Str1中那样初始化它
  char Str2[8] = {'a', 'r', 'd', 'u', 'i', 'n', 'o'};        //声明一个字符数组（带有一个额外的字符），编译器将添加所需的空字符，如Str2中所示
  char Str3[8] = {'a', 'r', 'd', 'u', 'i', 'n', 'o', '\0'};  //明确添加空字符Str3
  char Str4[ ] = "arduino";                      ";          //使用显式大小和字符串常量Str5初始化数组
  char Str6[9] = "arduino";                                  //初始化数组，为更大的字符串Str6留出额外的空间,空终止

char myString [] ="这是第一行"
                  "这是第二行";
char * myStrings [] = {"字符串1","字符串2","字符串3","字符串4"};  //*为数组指针
       myStrings [i] 
 */

