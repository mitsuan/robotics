void change_row()
{

  if(i==5)
  {
    goToDest();
  }

    //incrementing row number
//    i++;
     
  if(orient==0)
  {
    turn_right();
    orient=3;
  }
  else if(orient==2)
  {
    turn_left();
    orient=3;
  }

}

int search_x()
{
  stop1();
  delay(500);
  int turn;
  for(int m=0;m<6;m++)
  {
    if(a[i][m]==x)
    {
      if(m<j)
      turn=1;
      else if(m>j)
      turn=0;
      else
      {
        turn=-1;
      }
      break;
    }
  }

  return turn;
}


void goToDest()
{
  if(orient==3)
  {
    turn_left();
   
  }
  else if(orient==2)
  {
    turn_back();
  }

  while(j!=5)
    {
      move_forward();

      if(junction())
      {
        j++;
        while(junction())
        forward();
        
      }
    }

    stop1();
    while(1);
    
}

