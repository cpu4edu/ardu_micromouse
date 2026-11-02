//**************************************************************************
//			                                                                  	
//                  KNCT-MMEdu for Arduino       熊本高専 葉山清輝
// 
//                                                                                                 	
//　セットスイッチでメニュー選択．00-11までLEDに表示される       
//  00：センサチェック，01：予備，10：拡張左手法，11:最短走行       
//  2013.03.04  シンガポール学生との交流プログラム用 
//  1/18ギヤダウンステッピングモータ
//  フォトリフレクタ、LBR-127HLD使用・・・センサ感度が低いためプログラム改変
//
//**************************************************************************
#include <TimerOne.h>

#define PT_FR  0                         // 右前センサ，アナログ入力
#define PT_FL  1                         // 左前センサ，アナログ入力
#define PT_R   2                         // 右センサ，　アナログ入力
#define PT_L   3                         // 左センサ，　アナログ入力
#define GYRO   4                         // ジャイロ用に予約，アナログ入力
#define LEDOUT 19                        // 壁センサ用LED出力（AN5をデジタル19として使う）
#define SET    2                         // セットスイッチ，デジタル入力
#define START  3                         // スタートスイッチ，デジタル入力 

// 各種基準値
#define LSPD    4                       // 低速走行のタイマーカウント値
#define HSPD    3                       // 高速走行のタイマーカウント値
#define STEP1   520                      // １ステップの歩数 
#define SLOWSTP 20                       // スロースタートで加速に要するステップ数 
#define R90     205                      // 右９０度回転の歩数 
#define L90     205                      // 左９０度回転の歩数 
#define R180    410                      // 右１８０度回転の歩数 
#define DISFR   30                      // 前右壁の標準距離のセンサ値 
#define DISFL   32                      // 前左壁の標準距離のセンサ値 
#define DISR    11                       // 右壁の標準距離のセンサ値 
#define DISL    14                      // 左壁の標準距離のセンサ値    
#define DISFMAX 10                      // 前壁の最大距離のセンサ値(センサ値は最小) 
#define DISRMAX 4                       // 右壁の最大距離のセンサ値(センサ値は最小) 
#define DISLMAX 6                       // 左壁の最大距離のセンサ値(センサ値は最小) 

// モータの回転パターンテーブル，右と左は逆向きなので逆回転
const byte RMOTOR[]={0x03, 0x06, 0x0C, 0x09, 0x00};  // 右モータはPORTBへ出力
const byte LMOTOR[]={0x90, 0xC0, 0x60, 0x30, 0x00};  // 左モータはPORTDへ出力

const byte DtoR[]={0,2,4,0,8,0,0,0,1};   // 現在の方向に対して右側の方向を与えるテーブル 
const byte DtoL[]={0,8,1,0,2,0,0,0,4};   // 現在の方向に対して左側の方向を与えるテーブル       

byte pmode=0;                            // プログラムモード

// 割込み内で値を変化させる変数はVolatileで宣言が必要
volatile byte modeR=0, modeL=0;                   // モータの加減速・正逆転のモード  
volatile int stepR, stepL;                        // モータの回転数の指示用変数  
volatile unsigned char patR=0, patL=0;                     // モータの励磁パターンの位相 
volatile int cntR, cntL;                          // モータの回転数カウント 
volatile int sensFR, sensFL, sensR, sensL;        // センサの値 
volatile int sensFRB, sensFLB, sensRB, sensLB;    // LED消灯時のセンサの値 
volatile byte timR=0, timL=0;                     // 割り込み内で使うR,Lモータのタイマー 
volatile byte timS;                               // 割り込み内で使う光センサのタイマー 
volatile byte fS=0;                               // 速度フラグ，高速の時に壁との距離補正をかける  
volatile byte fR=0, fL=0;                         // R,Lモータの低速・高速切り替えフラグ 0:低速，1:高速  

union {                                  // マップを記憶する配列の構造定義  
       byte all;                         // mmapをバイト単位でアクセス  
       struct { byte n:1;                // 北方向の壁の有無（0:無，１：有）  
                byte e:1;                // 東方向の壁の有無（0:無，１：有）  
                byte s:1;                // 南方向の壁の有無（0:無，１：有）  
                byte w:1;                // 西方向の壁の有無（0:無，１：有）  
                byte d:4;                // 履歴（離脱方向）を４ビットで記憶領域確保  
              };
        } mmap[16][16]; 

//-------------------------------------------------------------------------- 
//  LED表示	                                              	 
//-------------------------------------------------------------------------- 
void dispLED(byte n)
{
 PORTB &= B11001111;                     // LEDを一旦消す
 PORTB |= (n<<4);                        // LED表示
}

//-------------------------------------------------------------------------- 
//  タイマー割り込み処理	                                              	 
//-------------------------------------------------------------------------- 
void SensAndMotor(void) {    

// モータ回転 mode = 0: フリー，1: 前進，2: 後退，3: ブレーキ
// 右モータ処理 
  if (timR>0) timR--;                         // timRをカウントダウン，timR=0のとき処理
  if (timR==0) {
    if (fR==0) timR=LSPD; else timR=HSPD;
    if (modeR==1) {if (patR < 3) patR++; else patR = 0; }
    if (modeR==2) {if (patR > 0) patR--; else patR = 3; }     
    cntR++;                                   // 右モータの歩数カウント 
  }
// 左モータ処理
  if (timL>0) timL--;                         // timLをカウントダウン，timL=0のとき処理
  if (timL==0) {
    if (fL==0) timL=LSPD; else timL=HSPD;
    if (modeL==1) {if (patL < 3) patL++; else patL = 0; }
    if (modeL==2) {if (patL > 0) patL--; else patL = 3; }          
    cntL++;                                   // 左モータの歩数カウント 
  }
    
  if (modeR==0 || modeL==0) { patR=4; patL=4; }  // mode0のときはフリー    
  PORTB &=0xf0; PORTB |= RMOTOR[patR];        // 右モータへパターンの出力
  PORTD &=0x0f; PORTD |= LMOTOR[patL];        // 左モータへパターンの出力

// センサ処理
// LED消灯時の基準値を取り，LED点灯後の差分でセンサ値を読み取る．
  if (timS<20) timS++; else timS=0;          // センサ読み取り周期カウンタ  
  if (timS==0){ 
    sensFRB=analogRead(PT_FR);               // LED消灯時の初期値を入力
    sensFLB=analogRead(PT_FL);
    sensRB=analogRead(PT_R);
    sensLB=analogRead(PT_L); 
    digitalWrite(LEDOUT, HIGH);              // LED-ON  
    delayMicroseconds(50);                   // LEDの点灯に時間がかかるので少し待つ
    sensFR=abs(analogRead(PT_FR)-sensFRB);        // LED消灯時の初期値を入力 
    sensFL=abs(analogRead(PT_FL)-sensFLB);
    sensR =abs(analogRead(PT_R) -sensRB);
    sensL =abs(analogRead(PT_L) -sensLB);  
    digitalWrite(LEDOUT, LOW);               // LED-OFF
  }  

// 左右センサによる距離の補正
// 右壁があるときは右壁のみを使い左壁無視，左壁のみの時は左壁だけをつかう
// 標準の距離（センサ値）と現在の差をとり，差が大きい時に補正する
  if (fS==1){                                   // 壁との距離補正する場合，以下の処理
    fR=fL=1;                                    // 左右をまず高速設定
    if(sensR>DISRMAX){                          // 右壁が存在しているなら右壁のみで調整  
      if ((sensR-DISR)>2) fL=0;                // 右壁に近づきすぎた時，左モータを減速
      if ((sensR-DISR)<-2) fR=0;               // 右壁から遠すぎる時，右モータを減速      
    } else if(sensL>DISLMAX){                   // 左壁のみ存在している場合の調整  
      if ((sensL-DISL)>2) fR=0;                // 考え方は右壁と同じ  
      if ((sensL-DISL)<-2) fL=0;
    }
  } else { fR=fL=0; }                           // fS=0の時，低速に設定

}

//-------------------------------------------------------------------------- 
// センサ調整，センサの値をLEDに表示		                                                                                             
//-------------------------------------------------------------------------- 
void check_sens() {  
  while (1){   
    Serial.write(0x0c);  // 改ページ
    Serial.print("Sensor FR:"); Serial.println(sensFR); // 右前センサ値を出力  
    Serial.print("Sensor FL:"); Serial.println(sensFL); // 左前センサ値を出力  
    Serial.print("Sensor  R:"); Serial.println(sensR);  // 右センサ値を出力  
    Serial.print("Sensor  L:"); Serial.println(sensL);  // 左センサ値を出力 
    delay (1000);
  }
}

//-------------------------------------------------------------------------- 
//	ブレーキ                        		                                     
//-------------------------------------------------------------------------- 
void run_break(){
  modeR=0; modeL=0;                     // モータ停止  
}

//-------------------------------------------------------------------------- 
//	前壁による角度調整         		                                           
//-------------------------------------------------------------------------- 
void adjust(){
  fS=0;                                 // 低速   
  while(abs((sensFR-DISFR)-(sensFL-DISFL))>20){  // 前センサの値の差が大きい時補正する
    if ((sensFR-DISFR)>(sensFL-DISFL)) { 
      modeR=2; modeL=1;                 // 右ターン
    } else { 
      modeR=1; modeL=2;                 // 左ターン
    } 
  } 
  run_break();
}

//-------------------------------------------------------------------------- 
//	前進時に低速でスタート                
//-------------------------------------------------------------------------- 
void slow_start(){
  fS=0;                                 // 最初は低速設定
  modeR=modeL=1;                        // モード設定 右:前進，左：前進  
  cntR=0; stepR=SLOWSTP;               // 低速でSLOWSTPで定義されたステップ数だけ進む
  while (cntR<stepR);                  // 指定回転数まで待つ． 
}

//-------------------------------------------------------------------------- 
//	ステップ前進                
//-------------------------------------------------------------------------- 
void run_step(){
  slow_start();
  fS=1;                                 // 残りの距離を高速で進む
  cntR=0; stepR=STEP1-SLOWSTP;             
  while (cntR<stepR);
  run_break();
}  

//-------------------------------------------------------------------------- 
//	右９０度回転          		                                     
//-------------------------------------------------------------------------- 
void run_R90(){ 
  fS=0;                                 // 低速設定 
  cntR=0; stepR=R90;                    // 右90度回転の歩数セット  
  modeR=2; modeL=1;                     // 右:後退，左：前進   
  while (cntR<stepR);
  run_break();
}  

//-------------------------------------------------------------------------- 
//	左９０度回転          		                                     
//-------------------------------------------------------------------------- 
void run_L90(){
  fS=0;                                 // 低速設定  
  cntL=0; stepL=L90;                    // 左90度回転の歩数セット  
  modeR=1; modeL=2;                     // 右:前進，左：後退  
  while (cntL<stepL);
  modeR=0; modeL=0;
  run_break();
}  

//-------------------------------------------------------------------------- 
//	右１８０度回転          		                                     
//-------------------------------------------------------------------------- 
void run_R180(){ 
  fS=0;                                 // 低速設定 
  cntR=0; stepR=R180;                   // 右90度回転の歩数セット  
  modeR=2; modeL=1;                     // 右:後退，左：前進   
  while (cntR<stepR);
  run_break();
}  

//--------------------------------------------------------------------------
//	Uターンまたは右ターン走行,加速前進，壁があったら減速，距離補正後ターン
//--------------------------------------------------------------------------
void run_Turn(byte n){
  while (1){
    slow_start();
    fS=1;                               // 残りの距離を高速で進む
    while (sensFR<DISFR);              // 前（右）センサで標準距離まで走らせる   
    adjust();                           // 前壁による角度調整   
    if (n==0) run_R180(); else run_R90();   // 右180度ターン または 右９０度ターン
  }
} 

//-------------------------------------------------------------------------- 
//	拡張左手法（履歴を記録）                                                 
//  左，前，右の優先順位で方向決定，行き止まりならUターン                    
//-------------------------------------------------------------------------- 
void run_Hidarite(){
  byte wF, wR, wL;               // 前，右，左に壁があるかどうかのフラグ  
  byte wS;                       // 壁センスを行ったかどうかのフラグ  
  byte mapF, mapR, mapL;         // マップ上の履歴を読み出す時に使う変数 
  byte mx,my;                    // マウスのx,y座標，初期は0,0，北にｙが増分，東にｘが増分  
  byte md;                       // マウスの向いている方向，北：１，東：２，南：４，西：８ 
  
  mapF=0; mapR=0; mapL=0;                     // マップ読み出し変数の初期化 
  mx=0; my=0; md=1;                           // マウスの初期座標は0,0, 北向き  
  wF=0; wR=1; wL=1;                           // スタート時点は前進から入る  
  mmap[0][0].d=1;                              // 初期位置の履歴を北方向にセット 

  while (digitalRead(START)==HIGH){                  // 青ボタンが押されるまで繰り返す  

  // 履歴の読み出し（mapF,mapR,mapLに前区画，右区画，左区画）
  // ただし，現在地が外周の区画のときはmmap配列からはみ出した区画をアクセスさせない．
    switch (md){
      case 1: if (my<15) mapF=mmap[my+1][mx].d; // 現方向が北向きのとき
              if (mx<15) mapR=mmap[my][mx+1].d; 
              if (mx>0)  mapL=mmap[my][mx-1].d; 
              break;
      case 2: if (mx<15) mapF=mmap[my][mx+1].d; // 現方向が東向きのとき
              if (my>0)  mapR=mmap[my-1][mx].d; 
              if (my<15) mapL=mmap[my+1][mx].d; 
              break;
      case 4: if (my>0)  mapF=mmap[my-1][mx].d; // 現方向が南向きのとき
              if (mx>0)  mapR=mmap[my][mx-1].d; 
              if (mx<15) mapL=mmap[my][mx+1].d; 
              break;     
      case 8: if (mx>0)  mapF=mmap[my][mx-1].d; // 現方向が西向きのとき
              if (my<15) mapR=mmap[my+1][mx].d; 
              if (my>0)  mapL=mmap[my-1][mx].d; 
              break;
    }
  
// ここから左手法の条件判断
    if (wL ==0 && (mapL==0 || mapL==DtoL[md]))      // 左壁が無く,未進入または履歴で左折可の時に左折  
      {run_L90(); md=DtoL[md]; }               
    else if (wF==0 && (mapF==0 || mapF==md)){}      // 前壁が無く,進入可のとき前進(elseif以下素通り) 
    else if (wR==0 && (mapR==0 || mapR==DtoR[md]))
      {run_R90(); md=DtoR[md]; }                    // 右壁が無く,進入可の時右折  
    else {run_R180(); md=DtoR[md]; md=DtoR[md];}    // Uターン  

// ここから，センサで前後左右の壁を見ながら前進させる．
    wS=0; wF=0; wR=0; wL=0;                         // 壁判断のフラグをリセット 
    slow_start();                                   // 低速スタート
    stepR=STEP1-SLOWSTP;
    fS=1;                                           // 高速設定
    while (cntR<stepR){                             // １ステップ分回す．   
      if (cntR > (STEP1*2/3) && wS==0){             // 歩数誤差による誤動作防止のためステップの2/3の所で壁検出    
        wS=1;                                       // 1度だけ壁検出するためフラグ（wS）を立てる                               
        if (sensR > DISRMAX) wR=1; else wR=0;       // 右壁検出   
        if (sensL > DISLMAX) wL=1; else wL=0;       // 左壁検出
        if ((sensFR>DISFMAX && sensFL>DISFMAX)){ wF=1; break; }  // 前壁検出，前壁があったらループを抜ける  
      }
    } 
    
    // 前壁を検出してループを抜けたとき，前壁まで進む
    if (wF==1){                                     
      while (sensFR<DISFR && sensFL<DISFL);                // 壁で距離調整 
      adjust();                                     // 前壁による角度調整
    } 

// 履歴（離脱方向の逆方向）を進む前の座標に記録．
// 壁情報はマウスの方向に応じて座標を更新した後に記録．                                      
    switch (md){  
      case 1: mmap[my][mx].d=4; my++; mmap[my][mx].n=wF; mmap[my][mx].e=wR; mmap[my][mx].w=wL; break; 
      case 2: mmap[my][mx].d=8; mx++; mmap[my][mx].e=wF; mmap[my][mx].s=wR; mmap[my][mx].n=wL; break;
      case 4: mmap[my][mx].d=1; my--; mmap[my][mx].s=wF; mmap[my][mx].w=wR; mmap[my][mx].e=wL; break;
      case 8: mmap[my][mx].d=2; mx--; mmap[my][mx].w=wF; mmap[my][mx].n=wR; mmap[my][mx].s=wL; break;
    } 
    if (mx==0 && my==0) { run_break(); break; }           // スタート地点に戻ったら探索終わり  
  }
}

//-------------------------------------------------------------------------- 
//	マップから最短距離を求め，最短走行                                       
//-------------------------------------------------------------------------- 
void run_saitan(){
  byte i,j,k,m;                  // 汎用変数 
  byte smap[16][16];             // 最短距離を求めるマップ  
  byte run[256];                 // 最短走行パターンを入れる配列 
  byte md;                       // マウスの向いている方向，北：１，東：２，南：４，西：８ 
    
// 最短距離マップをクリア，未探索区画は全てかべがあるものとする．
  for(i=0;i<16;i++){
    for(j=0;j<16;j++){         
      smap[i][j]=0;
      if (mmap[i][j].d==0){ 
        mmap[i][j].n=1; if (i<15) mmap[i+1][j].s=1;
        mmap[i][j].e=1; if (j<15) mmap[i][j+1].w=1;
        mmap[i][j].s=1; if (i>0)  mmap[i-1][j].n=1;
        mmap[i][j].w=1; if (j>0)  mmap[i][j-1].e=1;
      }
    }
  }

// 歩数マップ作成
// ゴール位置に１をセット，mの初期値は１. 全区画スキャンし，歩数ｍの位置なら移動可能な
// 区画に次の歩数（m+1）をセット.これを繰り返し，スタート地点に到達したらループを抜ける.
  
  smap[7][7]=1; smap[7][8]=1; smap[8][7]=1; smap[8][8]=1; // ゴールに１をセット
  m=1;                                                    // mの初期値セット  
  for(k=0;k<255;k++){                                     // 最高255回繰り返し  
    for(i=0;i<16;i++){
      for(j=0;j<16;j++){                                  // 全区画スキャン  
        if (smap[i][j]==m){                                 
          if (mmap[i][j].n==0 && i<15 && smap[i+1][j]==0) smap[i+1][j]=m+1;
          if (mmap[i][j].e==0 && j<15 && smap[i][j+1]==0) smap[i][j+1]=m+1;
          if (mmap[i][j].s==0 && i>0  && smap[i-1][j]==0) smap[i-1][j]=m+1;
          if (mmap[i][j].w==0 && j>0  && smap[i][j-1]==0) smap[i][j-1]=m+1;
        }
      }
    }
    m++;                                      // 歩数を進める 
    if (smap[0][0]!=0) break;                 // スタート地点まで歩数を埋めたらループから抜ける  
  }
        
// 歩数マップを逆にたどってrun[k] に最短経路の走行パターンを作る
// kはパターン数．1:直進, 2:右折，3:左折
  m=smap[0][0]-1;                             // mにスタート位置の歩数を入れ，逆にたどる  
  i=0; j=0; k=0;
  md=1;
  while (m>0){                                // ゴールまで到達した後，m--された時点で終わり  
    switch(md){
      case 1: if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=1; i++; m--;    break;}
              if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=3; md=DtoL[md]; break;}
      case 2: if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=1; j++; m--;    break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=3; md=DtoL[md]; break;}
      case 4: if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=1; i--; m--;    break;}
              if (mmap[i][j].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].e==0 && smap[i][j+1]==m && j<15) {run[k]=3; md=DtoL[md]; break;}
      case 8: if (mmap[i][j].w==0 && smap[i][j-1]==m && j>0 ) {run[k]=1; j--; m--;    break;}
              if (mmap[i][j].n==0 && smap[i+1][j]==m && i<15) {run[k]=2; md=DtoR[md]; break;}
              if (mmap[i][j].s==0 && smap[i-1][j]==m && i>0 ) {run[k]=3; md=DtoL[md]; break;}
    }
    k++;
  }
    
// 最短経路を走行
  i=0;
  while (i<k){
    if (run[i]==1) { run_step(); i++; }
    if (run[i]==2) { run_R90();  i++; }
    if (run[i]==3) { run_L90();  i++; }
  }
}

//--------------------------------------------------------------------------
//		初期化 		                                                  
//--------------------------------------------------------------------------
void setup()
{
  int i,j;

  DDRB |=B00111111;                      // PB0-4 右モータ，PB5,PB6はLEDのL0,L1
  DDRD |=B11110000;                      // PD4-7 左モータ, PD1はTX，PD2はSET,PD3はSTART
  pinMode(LEDOUT, OUTPUT);             // 壁センサ用LED出力
  digitalWrite(LEDOUT, LOW);           // 最初はLED消灯

// マップの初期化
  for (i=0; i<16;i++) for (j=0;j<16;j++) mmap[i][j].all=0;   // まずクリア
  for (i=0; i<16;i++){
     mmap[i][0].w=1; mmap[i][15].e=1;                         // 東西の外周の壁をセット 
     mmap[0][i].s=1; mmap[15][i].n=1; }                       // 南北の外周の壁をセット 
  mmap[0][0].e=1;    
 
  Timer1.initialize(1000);                 // 周期設定，1ms 
  Timer1.attachInterrupt(SensAndMotor);    // タイマースタート                   
  Serial.begin(9600);                  // シリアルポートを開く，壁センサチェック用
}

//--------------------------------------------------------------------------
//		メインループ 		                                                  
//--------------------------------------------------------------------------
void loop()
{
  while (digitalRead(START)==HIGH){
    if (digitalRead(SET)==LOW) {      // セットスイッチが押されたらプログラムモードを進める
      delay(10);  // チャタリング防止
      while(digitalRead(SET)==LOW);
      delay(10);  // チャタリング防止
      pmode++; if (pmode>3) pmode=0;
    }
    dispLED(pmode);
  }
  dispLED(0);
  delay(500);                           // 0.5秒待つ

  // プログラム実行
  switch(pmode){
/* 
    case  0: run_step(); break;          // １ステップ走行
    case  1: run_R90();  break;          // 右90度ターン 
    case  2: run_L90();  break;          // 左90度ターン    
    case  3: run_R180(); break;          // 180度ターン       
 */  
    case  0: check_sens();   break;       // センサチェック   
    case  1: run_Turn(0);    break;       // Uターン走行
    case  2: run_Hidarite(); break;       // 拡張左手法   
    case  3: run_saitan();   break;       // 最短経路を走行 
  }
}

