// ����ͷ�ļ�
//
//

       //Ŀ���ٶ�
//�� 6500 �ұ� 2000  �м䣺4300 //������ 3500-4500  ��->��
  
#define MAXELECTRICITY 2000      //��������������


#ifdef ROBOT1

  //�� 5300 �ұ� 3000  �м䣺4350
  #define YAWANGLEMAX   5900          //��ƫ�����
  #define YAWANGLEMIN   2100          //��ƫ���ұ�
  
  #define YAWANGLEMID   3800      //4250

  #define PITCHANGLEMAX   6700          //���������Ƕ�
  #define PITCHANGLEMIN   7450          //��������С�Ƕ�
  #define PITCHANGLEMID   7200
  
  #define CHASSISMAX     2600           //�������̵������������
  

#elif  ROBOT2
//�� 5300 �ұ� 3000  �м䣺4350
  #define YAWANGLEMAX   3000          //��ƫ���ұ�
  #define YAWANGLEMIN   5300          //��ƫ�����
  #define YAWANGLEMID   4400 

  #define PITCHANGLEMAX   4800       //���������Ƕ�
  #define PITCHANGLEMIN   3900       //��������С�Ƕ�
  #define PITCHANGLEMID   4450

  #define CHASSISMAX     3500
    
#elif  ROBOT3

   //�� 5300 �ұ� 3000  �м䣺4350
  #define YAWANGLEMAX   4700          //��ƫ�����
  #define YAWANGLEMIN   200          //��ƫ���ұ�
  #define YAWANGLEMID   2400 


  #define PITCHANGLEMAX   500       //���������Ƕ�
  #define PITCHANGLEMIN   1140       //��������С�Ƕ�
  #define PITCHANGLEMID   650
  #define CHASSISMAX     2500

#endif



#define INTEGARALSNUM    5        //PID���ִ���

#define MOTORID   0x200  //���̵��ID
#define HEADID    0x1FF   //��̨���ID  
 
 //uint8_t sdm;

#define CHEAKSTART     0X00  //У����ʼ
#define CHEAKING       0X40  //У��������
#define CHEAKFINISH    0X80  //У������


