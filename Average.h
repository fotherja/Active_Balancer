#ifndef _AVERAGE_H_
#define _AVERAGE_H_

class Average {
  public:
    Average();
    float Rolling_Average(float Value);
  
  private:
    float Num_Buffer[FILTER_SIZE];
    unsigned char Index;
};

Average::Average()
{
  Index = 0; 

  for(byte i = 0;i < FILTER_SIZE;i++)
    Num_Buffer[i] = 0;
}

float Average::Rolling_Average(float Value)
{
  Num_Buffer[Index] = Value;                                      // Put new value into buffer array.
  Index++;
    
  if(Index == FILTER_SIZE)                                        // Roll back Index if it overflows
    Index = 0;

  float sum = 0;
  for(byte i = 0;i < FILTER_SIZE;i++)
    sum += Num_Buffer[i];

  return(sum/FILTER_SIZE);
}

#endif
