int phi(int n)
{
    if(n==0)
    {
      Serial.print("y= ");
      Serial.println(0);
      return 0;
    }
    else{
    float result = n;   // Initialize result as n

    // Consider all prime factors of n and for every prime
    // factor p, multiply result with (1 - 1/p)
    for (int p=2; p*p<=n; ++p)
    {
        // Check if p is a prime factor.
        if (n % p == 0)
        {
            // If yes, then update n and result
            while (n % p == 0)
                n /= p;
            result *= (1.0 - (1.0 / (float) p));
        }
    }

    // If n has a prime factor greater than sqrt(n)
    // (There can be at-most one such prime factor)
    if (n > 1)
        result *= (1.0 - (1.0 / (float) n));

    Serial.print("y= ");
    Serial.println((int)result);
    return (int)result;
    }
}

void update_coord()
{
  switch(orient)
  {
    case 0:x++;break;
    case 1:y++;break;
    case 2:x--;break;
    case 3:y--;
  }
}

