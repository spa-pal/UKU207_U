					else if(ind==iAusw)
	{
	if(AUSW_MAIN==6024)		ptrs[0]="   ИБЭП220/60-24A   ";
	else if(AUSW_MAIN==6012)	ptrs[0]="   ИБЭП220/60-12A   ";
	else if(AUSW_MAIN==4824)	ptrs[0]="   ИБЭП220/48-24A   ";
	else if(AUSW_MAIN==4812)	ptrs[0]="   ИБЭП220/48-12A   ";
	else if(AUSW_MAIN==6010)	ptrs[0]="   ИБЭП220/60-10A   ";
	else if(AUSW_MAIN==6005)	ptrs[0]="   ИБЭП220/60-5A    ";
	else if(AUSW_MAIN==4810)	ptrs[0]="   ИБЭП220/48-10A   ";
	else if(AUSW_MAIN==4805)	ptrs[0]="   ИБЭП220/48-5A    ";
	else if(AUSW_MAIN==2424)	ptrs[0]="   ИБЭП220/24-24A   ";
	else if(AUSW_MAIN==2412)	ptrs[0]="   ИБЭП220/24-12A   ";
	else if(AUSW_MAIN==4840)	ptrs[0]="   ИБЭП220/48-40A   ";
	else if(AUSW_MAIN==6030)	ptrs[0]="   ИБЭП220/60-30A   ";
	else if(AUSW_MAIN==4820)	ptrs[0]="   ИБЭП220/48-20A   ";
	else if(AUSW_MAIN==6015)	ptrs[0]="   ИБЭП220/60-15A   ";
	else if(AUSW_MAIN==2450)	ptrs[0]="   ИБЭП220/24-50A   ";
	else if(AUSW_MAIN==2425)	ptrs[0]="   ИБЭП220/24-25A   ";	
	else if(AUSW_MAIN==2424)	ptrs[0]="   ИБЭП220/24-24A   ";
	else if(AUSW_MAIN==2412)	ptrs[0]="   ИБЭП220/24-12A   ";

	else ptrs[0]="   Тип неизвестен   ";
	
	ptrs[1]="Дата изгот. 0!/0@/0#";
	
	ptrs[2]="шасси     S/N00000? ";
	
	ptrs[3]="УКУ202.+( S/N00000) ";
	
	ptrs[4]="ПО        v    [    ";
	
	ptrs[5]="БПС1 -    S/N00000< ";
	
	ptrs[6]="БПС2 -    S/N00000> ";
	
	ptrs[7]="RS232      v   $    ";

	ptrs[8]="CAN PDH -  v   %    ";
	
	ptrs[9]="CAN SDH -  v   ^    ";
	
	ptrs[10]="Ethernet-  v   &    ";

	ptrs[11]=" Выход              ";		
	
	
	bgnd_par(ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);	
	if(index_set==8)lcd_buffer[60]=1;
	/*long2lcdyx_mmm(AUSW_BPS2_NUMBER,3,9,0);
	long2lcdyx_mmm(lc640_read_long(EE_AUSW_BPS2_NUMBER),3,19,0);*/
	
	if(AUSW_DAY<32)int2lcd(AUSW_DAY,'!',0);
	else sub_bgnd("**",'!',-1);
	if(AUSW_MONTH<13)int2lcd(AUSW_MONTH,'@',0);
	else sub_bgnd("**",'@',-1);
	if(AUSW_YEAR<100)int2lcd(AUSW_YEAR,'#',0);
	else sub_bgnd("**",'#',-1);	

	if(AUSW_MAIN_NUMBER>=1000000)sub_bgnd("******",'?',-5);
	else long2lcd_mmm(AUSW_MAIN_NUMBER,'?',0);

	/*if(AUSW_UKU>=1000) sub_bgnd("***",'+',-2);
	else int2lcd(AUSW_UKU,'+',0);

	if(AUSW_UKU_SUB>=100) sub_bgnd("**",'(',-1);
	else int2lcd(AUSW_UKU_SUB,'(',0);*/
	int2lcd(UKU_VERSION/100,'+',0);
	int2lcd(HARDVARE_VERSION,'(',0);
	
	if(AUSW_UKU_NUMBER>=1000000)sub_bgnd("******",')',-5);
	else long2lcd_mmm(AUSW_UKU_NUMBER,')',0);

	if(AUSW_UKU_NUMBER>=1000000)sub_bgnd("******",')',-5);
	else long2lcd_mmm(AUSW_UKU_NUMBER,')',0);
	
	if(AUSW_BPS1_NUMBER>=1000000)sub_bgnd("******",'<',-5);
	else long2lcd_mmm(AUSW_BPS1_NUMBER,'<',0);	
	
	if(AUSW_BPS2_NUMBER>=1000000)sub_bgnd("******",'>',-5);
	else long2lcd_mmm(AUSW_BPS2_NUMBER,'>',0);	

	#ifdef SOFT 
	int2lcd(SOFT,'[',2);
	#else 
	sub_bgnd("---------",'[',-4);
	#endif

	#ifdef RS232_VERSION 
	int2lcd(RS232_VERSION,'$',2);
	#else 
	sub_bgnd("---------",'$',-4);
	#endif
	
	#ifdef CAN_PDH_VERSION 
	int2lcd(CAN_PDH_VERSION,'%',2);
	#else 
	sub_bgnd("---------",'%',-4);
	#endif	
	//else if((AUSW_RS232<0)||(AUSW_RS232>=1000))sub_bgnd("не установлен",'$',-5);
	//else 

	//if(AUSW_PDH==0)sub_bgnd("отсутствует",'%',-5);
//	else if((AUSW_PDH<0)||(AUSW_PDH>=1000))sub_bgnd("не установлен",'%',-5);
	//else int2lcd(AUSW_PDH,'%',2);

	#ifdef CAN_SDH_VERSION 
	int2lcd(CAN_SDH_VERSION,'^',2);
	#else 
	sub_bgnd("---------",'^',-4);
	#endif	
	
	//if(AUSW_SDH==0)sub_bgnd("отсутствует",'%',-5);
	//else if((AUSW_SDH<0)||(AUSW_SDH>=1000))sub_bgnd("не установлен",'%',-5);
	//else int2lcd(AUSW_SDH,'%',2);

	#ifdef ETH_VERSION 
	int2lcd(ETH_VERSION,'&',2);
	#else 
	sub_bgnd("---------",'&',-4);
	#endif	
	
	fl_simv(0,0,0);
	
	}

else if(ind==iAusw_set)
	{
	if(AUSW_MAIN==6024)		ptrs[0]="   ИБЭП220/60-24A   ";
	else if(AUSW_MAIN==6012)	ptrs[0]="   ИБЭП220/60-12A   ";
	else if(AUSW_MAIN==4824)	ptrs[0]="   ИБЭП220/48-24A   ";
	else if(AUSW_MAIN==4812)	ptrs[0]="   ИБЭП220/48-12A   ";
	else if(AUSW_MAIN==6010)	ptrs[0]="   ИБЭП220/60-10A   ";
	else if(AUSW_MAIN==6005)	ptrs[0]="   ИБЭП220/60-5A    ";
	else if(AUSW_MAIN==4810)	ptrs[0]="   ИБЭП220/48-10A   ";
	else if(AUSW_MAIN==4805)	ptrs[0]="   ИБЭП220/48-5A    ";
	else if(AUSW_MAIN==2424)	ptrs[0]="   ИБЭП220/24-24A   ";
	else if(AUSW_MAIN==2412)	ptrs[0]="   ИБЭП220/24-12A   ";
	else if(AUSW_MAIN==4840)	ptrs[0]="   ИБЭП220/48-40A   ";
	else if(AUSW_MAIN==6030)	ptrs[0]="   ИБЭП220/60-30A   ";
	else if(AUSW_MAIN==4820)	ptrs[0]="   ИБЭП220/48-20A   ";
	else if(AUSW_MAIN==6015)	ptrs[0]="   ИБЭП220/60-15A   ";
	else if(AUSW_MAIN==2450)	ptrs[0]="   ИБЭП220/24-50A   ";
	else if(AUSW_MAIN==2425)	ptrs[0]="   ИБЭП220/24-25A   ";
		
	else ptrs[0]="   Тип неизвестен   ";
	
	ptrs[1]="Дата изгот. 0!/0@/0#";
	
	ptrs[2]="шасси     S/N00000? ";
	
	ptrs[3]="УКУ00+.0( S/N00000) ";
	
	ptrs[4]="БПС1 -    S/N00000< ";
	
	ptrs[5]="БПС2 -    S/N00000> ";
	
	ptrs[6]=" Выход              ";		
	
	
	bgnd_par(ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);	
	
	/*long2lcdyx_mmm(AUSW_BPS2_NUMBER,3,9,0);
	long2lcdyx_mmm(lc640_read_long(EE_AUSW_BPS2_NUMBER),3,19,0);*/

	if(AUSW_DAY<32)int2lcd(AUSW_DAY,'!',0);
	else sub_bgnd("**",'!',-1);
	if(AUSW_MONTH<13)int2lcd(AUSW_MONTH,'@',0);
	else sub_bgnd("**",'@',-1);
	if(AUSW_YEAR<100)int2lcd(AUSW_YEAR,'#',0);
	else sub_bgnd("**",'#',-1);	
	
	if(AUSW_MAIN_NUMBER>=1000000)sub_bgnd("******",'?',-5);
	else long2lcd_mmm(AUSW_MAIN_NUMBER,'?',0);
	
	/*if(AUSW_UKU>=1000) sub_bgnd("***",'+',-2);
	else int2lcd(AUSW_UKU,'+',0);

	if(AUSW_UKU_SUB>=100) sub_bgnd("**",'(',-1);
	else int2lcd(AUSW_UKU_SUB,'(',0);*/
	
	int2lcd(UKU_VERSION,'+',0);
	int2lcd(HARDVARE_VERSION,'(',0);	

	if(AUSW_UKU_NUMBER>=1000000)sub_bgnd("******",')',-5);
	else long2lcd_mmm(AUSW_UKU_NUMBER,')',0);

	if(AUSW_BPS1_NUMBER>=1000000)sub_bgnd("******",'<',-5);
	else long2lcd_mmm(AUSW_BPS1_NUMBER,'<',0);	

	if(AUSW_BPS2_NUMBER>=1000000)sub_bgnd("******",'>',-5);
	else long2lcd_mmm(AUSW_BPS2_NUMBER,'>',0);
	
				
	if((index_set==3)&&(sub_ind==9))lcd_buffer[60]=1;

	
	if((sub_ind==0)&&(!index_set)) fl_simv(0,0,20);	
	else if((sub_ind==1)&&(index_set<2))fl_simv(1-index_set,12,2);
	else if((sub_ind==2)&&(index_set<2))fl_simv(1-index_set,15,2);
	else if((sub_ind==3)&&(index_set<2))fl_simv(1-index_set,18,2);
	
	else if((sub_ind==4)&&(index_set<3)) fl_simv(2-index_set,10,10);	
	else if((sub_ind==5)&&(index_set<4)) fl_simv(3-index_set,3,6);	
	else if((sub_ind==6)&&(index_set<4)) fl_simv(3-index_set,10,10);	

	else if((sub_ind==7)/*&&(index_set<4)*/) fl_simv(4-index_set,10,10);	
	else if((sub_ind==8)/*&&(index_set<4)*/) fl_simv(5-index_set,10,10);	
	else fl_simv(0,0,0);	
	/*
	
	
	ptrs[1]="Дата изгот. - неизв.";
	if((AUSW_YEAR<=99)||(AUSW_MONTH<=12)||(AUSW_DAY<=31))
		{
		

		}	
	
	
	ptrs[2]="шасси S/N - неизв.  ";
	ptrs[2]="шасси S/N - неизв.  ";
	if(AUSW_MAIN_NUMBER<60000)
		{
		ptrs[2]="шасси S/N - 00000?  ";
		
		}	

	ptrs[3]="УКУ--+.-( S/N неуст)";
	if(AUSW_UKU>1000) sub_bgnd("-.--",'(',-3);
	else 
		{
	
		}
	if(AUSW_UKU_SUB>99) sub_bgnd("-",'(',0);
	else 
		{
	
		}		
	if(AUSW_UKU_NUMBER>30000)sub_bgnd("ст.",')',-2);
	else
		{
		sub_bgnd("00000)",')',-5);
		//int2lcd((signed short)AUSW_UKU_NUMBER,')',0);
		}

	ptrs[4]="БПС1 - не установлен";
	if(AUSW_BPS1<60000)
		{
		ptrs[4]="БПС1 - S/N0000<     ";
		
		}
		
	ptrs[5]="БПС2 - не установлен";	
	if(AUSW_BPS2<60000)
		{
		ptrs[5]="БПС2 - S/N0000>     ";
		
		}	

	if((AUSW_RS232==0)||(AUSW_RS232>=1000))
		{
		ptrs[6]="RS232   отсутствует ";
		}
	else ptrs[6]="RS232 - версия     [";
	
	if((AUSW_PDH==0)||(AUSW_PDH>=1000))
		{
		ptrs[7]="CAN PDH отсутствует ";
		}
	else ptrs[7]="CAN PDH - версия   ]";
	
	if((AUSW_SDH==0)||(AUSW_SDH>=1000))
		{
		ptrs[8]="CAN SDH отсутствует ";
		}
	else ptrs[8]="CAN SDH - версия   )";		
	
	
	int2lcd((unsigned short)AUSW_MAIN_NUMBER,'?',0);
	if(AUSW_DAY<32)int2lcd(AUSW_DAY,'!',0);
	else sub_bgnd("**",'!',-1);
	if(AUSW_MONTH<13)int2lcd(AUSW_MONTH,'@',0);
	else sub_bgnd("**",'@',-1);
	if(AUSW_YEAR<100)int2lcd(AUSW_YEAR,'#',0);
	else sub_bgnd("**",'#',-1);
	int2lcd((unsigned short)AUSW_BPS1,'<',0);
	int2lcd((unsigned short)AUSW_BPS2,'>',0);
	int2lcd(AUSW_UKU,'+',0);
	int2lcd(AUSW_UKU_SUB,'(',0);
	if(AUSW_UKU_NUMBER==0xffff)
		{
		sub_bgnd(" ",')',0);
		}
	else
		{
		sub_bgnd("     )",')',-5);
		int2lcd((unsigned short)AUSW_UKU_NUMBER,')',0);
		}
	
	//int2lcdyx(AUSW_UKU,0,19,0);
	
	if(ind==iAusw_set)
		{
		if((sub_ind==0)&&(!index_set)) fl_simv(0,0,20);
		
		else if((sub_ind==1)&&(index_set<2))
			{
			if((AUSW_YEAR>99)&&(AUSW_MONTH>12)&&(AUSW_DAY>31)) fl_simv(1-index_set,14,6);
			else fl_simv(1,12,2);
			}
		else if((sub_ind==2)&&(index_set<2))
			{
			if((AUSW_YEAR>99)&&(AUSW_MONTH>12)&&(AUSW_DAY>31)) fl_simv(1-index_set,14,6);
			else fl_simv(1,15,2);
			}	
		else if((sub_ind==3)&&(index_set<2))
			{
			if((AUSW_YEAR>99)&&(AUSW_MONTH>12)&&(AUSW_DAY>31)) fl_simv(1-index_set,14,6);
			else fl_simv(1,18,2);
			}	
		else if((sub_ind==4)&&(index_set<3)) fl_simv(2-index_set,10,10);	
		else if((sub_ind==5)&&(index_set<4)) fl_simv(3-index_set,3,6);	
		else if((sub_ind==6)&&(index_set<4)) fl_simv(3-index_set,14,6);				
		}*/
	}


else if(ind==iAusw)
	{
	if(but==butD)
		{
		index_set++;
		gran_char(&index_set,0,8);
		}
	else if(but==butU)
		{
		index_set--;
		gran_char(&index_set,0,8);
		}
	else if(but==butD_)
		{
		index_set=8;
		}				
	else if(but==butE_)
		{
		tree_up(iAusw_prl,0,0,0);
		ret(50);
		parol_init();
		}	
     else if(index_set==8)
	     {
	     if(but==butE)
	          {
	          tree_down(0,0);
	          ret(0);
	          }
	     }		
    else  if(but==butL)
	     {
          tree_down(0,0);
          ret(0);
	     }			     	
	}

else if(ind==iAusw_set)
	{
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==5)sub_ind++;
		if(sub_ind==6)sub_ind++;
		gran_char(&sub_ind,0,9);
		if((sub_ind>6)&&(index_set<1))index_set=1;
		if((sub_ind>7)&&(index_set<2))index_set=2;
		if((sub_ind>8)&&(index_set<3))index_set=3;
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==6)sub_ind--;
		if(sub_ind==5)sub_ind--;
		gran_char(&sub_ind,0,9);
		if((sub_ind<5)&&(index_set>2))index_set=2;
		if((sub_ind<4)&&(index_set>1))index_set=1;
		if((sub_ind<1)&&(index_set))index_set=0;
		}
	else if(but==butD_)
		{
		sub_ind=9;
		}			
	else if((but==butE_)||((but==butE)&&(sub_ind==9)))
		{
		fl_simv(0,0,0);
		tree_down(-1,0);
		ret(0);
		}
			
     else if(sub_ind==0)
	     {
/*	     if((but==butR)||(but==butR_))
	     	{
			if(AUSW_MAIN==4812)AUSW_MAIN=4824;
	          else if(AUSW_MAIN==4824)AUSW_MAIN=6012;
	          else if(AUSW_MAIN==6012)AUSW_MAIN=6024;
	          else AUSW_MAIN=4812;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }
	     else if((but==butL)||(but==butL_))
	          {
	          if(AUSW_MAIN==4812)AUSW_MAIN=6024;
	          else if(AUSW_MAIN==4824)AUSW_MAIN=4812;
	          else if(AUSW_MAIN==6012)AUSW_MAIN=4824;
	          else AUSW_MAIN=6012;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }*/

#if(UKU_VERSION==900)

#ifdef _24_


	     if((but==butR)||(but==butR_))
	     	{
			if(AUSW_MAIN==2450)AUSW_MAIN=2425;
	          else AUSW_MAIN=2450;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }
	     else if((but==butL)||(but==butL_))
	          {
			if(AUSW_MAIN==2450)AUSW_MAIN=2425;
	          else AUSW_MAIN=2450;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }

#else


	     if((but==butR)||(but==butR_))
	     	{
			if(AUSW_MAIN==6024)AUSW_MAIN=4824;
               else if(AUSW_MAIN==4824)AUSW_MAIN=6012;
               else if(AUSW_MAIN==6012)AUSW_MAIN=4812;
               else if(AUSW_MAIN==4812)AUSW_MAIN=6040;
               else if(AUSW_MAIN==6040)AUSW_MAIN=4840;
               else if(AUSW_MAIN==4840)AUSW_MAIN=6020;
               else if(AUSW_MAIN==6020)AUSW_MAIN=4820;
               else AUSW_MAIN=6024;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }
	     else if((but==butL)||(but==butL_))
	          {
			if(AUSW_MAIN==6024)AUSW_MAIN=4820;
               else if(AUSW_MAIN==4824)AUSW_MAIN=6024;
               else if(AUSW_MAIN==6012)AUSW_MAIN=4824;
               else if(AUSW_MAIN==4812)AUSW_MAIN=6012;
               else if(AUSW_MAIN==6040)AUSW_MAIN=4812;
               else if(AUSW_MAIN==4840)AUSW_MAIN=6040;
               else if(AUSW_MAIN==6020)AUSW_MAIN=4840;
               else AUSW_MAIN=6020;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }

#endif				
#endif 

#if(UKU_VERSION==300)
#ifdef _24_

	     if((but==butR)||(but==butR_))
	     	{
			if(AUSW_MAIN==2424)AUSW_MAIN=2412;
	          else AUSW_MAIN=2424;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }
	     else if((but==butL)||(but==butL_))
	          {
			if(AUSW_MAIN==2424)AUSW_MAIN=2412;
	          else AUSW_MAIN=2424;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }

#else

	     if((but==butR)||(but==butR_))
	     	{
			if(AUSW_MAIN==6010)AUSW_MAIN=4810;
               else if(AUSW_MAIN==4810)AUSW_MAIN=6005;
               else if(AUSW_MAIN==6005)AUSW_MAIN=4805;
               else AUSW_MAIN=6010;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }
	     else if((but==butL)||(but==butL_))
	          {
			if(AUSW_MAIN==6010)AUSW_MAIN=4805;
               else if(AUSW_MAIN==4810)AUSW_MAIN=6010;
               else if(AUSW_MAIN==6005)AUSW_MAIN=4810;
               else AUSW_MAIN=6005;
	          lc640_write_int(EE_AUSW_MAIN,AUSW_MAIN);
	          }


#endif				
#endif    
		}

	          	          
	    	
	     
     else if(sub_ind==1)
	     {
	     speed=1;
	     if(but==butR)
	          {
	          AUSW_DAY++;
	          }
	     else if(but==butR_)
	          {
	          AUSW_DAY+=2;
		     }
	     else if(but==butL)
	          {
	          AUSW_DAY--;
		     }
	     else if(but==butL_)
	          {
	          AUSW_DAY-=2;
		     }
		else if(but==butLR)
	          {
	          lc640_write_int(EE_AUSW_DAY,LPC_RTC->DOM);
	          lc640_write_int(EE_AUSW_MONTH,LPC_RTC->MONTH);
	          lc640_write_int(EE_AUSW_YEAR,LPC_RTC->YEAR);
	          sub_ind=4;
		     }     
		gran(&AUSW_DAY,1,31);     		     		     
		lc640_write_int(EE_AUSW_DAY,AUSW_DAY); 	
	     }
	     	
     else if(sub_ind==2)
	     {
	     speed=1;
	     if(but==butR)
	          {
	          AUSW_MONTH++;
	          }
	     else if(but==butR_)
	          {
	          AUSW_MONTH+=2;
		     }
	     else if(but==butL)
	          {
	          AUSW_MONTH--;
		     }
	     else if(but==butL_)
	          {
	          AUSW_MONTH-=2;
		     }
		else if(but==butLR)
	          {
	          lc640_write_int(EE_AUSW_DAY,LPC_RTC->DOM);
	          lc640_write_int(EE_AUSW_MONTH,LPC_RTC->MONTH);
	          lc640_write_int(EE_AUSW_YEAR,LPC_RTC->YEAR);
	          sub_ind=4;
		     }		     
		gran(&AUSW_MONTH,1,12);     		     		     
		lc640_write_int(EE_AUSW_MONTH,AUSW_MONTH);
	     }		
	     
     else if(sub_ind==3)
	     {
	     speed=1;
	     if(but==butR)
	          {
	          AUSW_YEAR++;
	          }
	     else if(but==butR_)
	          {
	          AUSW_YEAR+=5;
		     }
	     else if(but==butL)
	          {
	          AUSW_YEAR--;
		     }
	     else if(but==butL_)
	          {
	          AUSW_YEAR-=5;
		     }
		else if(but==butLR)
	          {
	          lc640_write_int(EE_AUSW_DAY,LPC_RTC->DOM);
	          lc640_write_int(EE_AUSW_MONTH,LPC_RTC->MONTH);
	          lc640_write_int(EE_AUSW_YEAR,LPC_RTC->YEAR);
	          sub_ind=4;
		     }		     
		gran(&AUSW_YEAR,1,12);     		     		     
		lc640_write_int(EE_AUSW_YEAR,AUSW_YEAR);
	     }		          			     		
	     
     else if(sub_ind==4)
	     {
	     speed=1;
	     if(but==butR)
	          {
	          AUSW_MAIN_NUMBER++;
	          }
	     else if(but==butR_)
	          {
	          AUSW_MAIN_NUMBER+=10;
		     }
	     else if(but==butL)
	          {
	          AUSW_MAIN_NUMBER--;
		     }
	     else if(but==butL_)
	          {
	          AUSW_MAIN_NUMBER-=10;
		     }
		gran_long(&AUSW_MAIN_NUMBER,9000,999999L);     		     		     
		lc640_write_long(EE_AUSW_MAIN_NUMBER,AUSW_MAIN_NUMBER);
		lc640_write_long(EE_AUSW_UKU_NUMBER,AUSW_MAIN_NUMBER);
	     }				     		

    else if(sub_ind==6)
	     {
     	speed=1;
	     if(but==butR)
	          {
	          AUSW_UKU_NUMBER++;
	          }
	     else if(but==butR_)
	          {
	          AUSW_UKU_NUMBER+=10;
		     }
	     else if(but==butL)
	          {
	          AUSW_UKU_NUMBER--;
		     }
	     else if(but==butL_)
	          {
	          AUSW_UKU_NUMBER-=10;
		     }
		gran_long(&AUSW_UKU_NUMBER,1,999999);     		     		     
		lc640_write_long(EE_AUSW_UKU_NUMBER,AUSW_UKU_NUMBER);
	     }
	     
  else if(sub_ind==7)
	     {
     	speed=1;
	     if(but==butR)
	          {
	          AUSW_BPS1_NUMBER++;
	          }
	     else if(but==butR_)
	          {
	          AUSW_BPS1_NUMBER+=10;
		     }
	     else if(but==butL)
	          {
	          AUSW_BPS1_NUMBER--;
		     }
	     else if(but==butL_)
	          {
	          AUSW_BPS1_NUMBER-=10;
		     }
		gran_long(&AUSW_BPS1_NUMBER,20000,999999);     		     		     
		lc640_write_long(EE_AUSW_BPS1_NUMBER,AUSW_BPS1_NUMBER);
	     }	     
	     
 else if(sub_ind==8)
	     {
     	speed=1;
	     if(but==butR)
	          {
	          AUSW_BPS2_NUMBER++;
	          }
	     else if(but==butR_)
	          {
	          AUSW_BPS2_NUMBER+=10;
		     }
	     else if(but==butL)
	          {
	          AUSW_BPS2_NUMBER--;
		     }
	     else if(but==butL_)
	          {
	          AUSW_BPS2_NUMBER-=10;
		     }
		gran_long(&AUSW_BPS2_NUMBER,20000,999999);     		     		     
		lc640_write_long(EE_AUSW_BPS2_NUMBER,AUSW_BPS2_NUMBER);
	     }	     	     
	     
else if(sub_ind==9)
	     {
     	speed=1;
	     if(but==butR)
	          {
	          AUSW_RS232++;
	          }
	     else if(but==butR_)
	          {
	          AUSW_RS232+=10;
		     }
	     else if(but==butL)
	          {
	          AUSW_RS232--;
		     }
	     else if(but==butL_)
	          {
	          AUSW_RS232-=10;
		     }
		gran(&AUSW_RS232,0,1000);     		     		     
		lc640_write_long(EE_AUSW_BPS2_NUMBER,AUSW_BPS2_NUMBER);
	     }	
	else if(sub_ind==10)
	     {
	     if(but==butE)
	     	{
	     	tree_down(-1,0);
	     	ret(0);
	     	}
	     }    	     	     
	}