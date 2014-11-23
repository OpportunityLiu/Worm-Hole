void setup()
{

  /* add setup code here */
    
}

void loop()
{
  /* add main program code here */
    for (size_t i = 0; i < 256; i++)
    {

        analogWrite(11, i);
        delay(10);
    }
}
