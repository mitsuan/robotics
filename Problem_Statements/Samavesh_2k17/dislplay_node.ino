//
//int numToDisp=0;
//
//void display(int n)
//{
//  numToDisp=n;
//  
//}
//void displayOnBoard()
//{
//  String nstr="";
//  if(String(numToDisp).length()==1)
//  {
//  nstr=String(0)+String(numToDisp);
//
//  digitalWrite(d0,HIGH);
//  digitalWrite(d1,LOW);
//  sym(nstr.charAt(1));
//  
//  }
//  else
//  {
//  nstr=String(numToDisp);
//
//  digitalWrite(d0,HIGH);
//  digitalWrite(d1,LOW);
//  sym(nstr.charAt(1));
//  delay(10);
//
//  digitalWrite(d1,HIGH);
//  digitalWrite(d0,LOW);
//  sym(nstr.charAt(0));
//  delay(10);
//  }
//
//  
//}
//
//void sym(char ch)
//{
//      switch(ch)
//    {
//      case '1':
//      {
//      digitalWrite(40,LOW);
//      digitalWrite(41,LOW);
//      digitalWrite(42,LOW);
//      digitalWrite(43,HIGH);
//      break;
//      }
//      case '2':
//      {
//        digitalWrite(40,LOW);
//        digitalWrite(41,LOW);
//        digitalWrite(42,HIGH);
//        digitalWrite(43,LOW);
//        break;
//      }
//      case '3':
//      {
//        digitalWrite(40,LOW);
//        digitalWrite(41,LOW);
//        digitalWrite(42,HIGH);
//        digitalWrite(43,HIGH);
//      }
//      case '4':
//      {
//        digitalWrite(40,LOW);
//        digitalWrite(41,HIGH);
//        digitalWrite(42,LOW);
//        digitalWrite(43,LOW);
//        break;
//      }
//      case '5':
//      {
//        digitalWrite(40,LOW);
//        digitalWrite(41,HIGH);
//        digitalWrite(42,LOW);
//        digitalWrite(43,HIGH);
//        break;
//      }
//      case '6':
//      {
//        digitalWrite(40,LOW);
//        digitalWrite(41,HIGH);
//        digitalWrite(42,HIGH);
//        digitalWrite(43,LOW);
//        break;
//      }
//  }
//}

