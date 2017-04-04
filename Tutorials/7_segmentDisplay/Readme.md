# Explanation
This is an example code for displaying digits on a 4 digit multiplexed 7 segment display board. 

## Details of the display board:
  The board has 4 '7-segment' displays 
  All their pins except the enable pin were shorted to use them in time multiplexed manner. 
  Four enable pins were provided to control each display unit independently 
  
## Working of the code

   -The code contains the format in which the segments need to be glowed in order show a particular number in the form of an array.
   -Another array of length 4 stores the digit to be displayed in the 4 display units.
   -In the loop function, the availability of serial data is checked continuously.
   -If a data is entered, the array which stores the digit to be dispayed is updated.
   -At last display function of each display unit is called with a delay of few ms.
   -In the display function of a particular display unit e.g. the 1st display unit, the digit to be displayed is passed. It first makes the enable pin of the rest of the display units i.e 2nd 3rd and 4th, HIGH (as providing HIGH signal to the enable pin makes them OFF) to make them OFF. Then it fetches the segments to be glowed in order to display the passed digit and makes the corresponding pins HIGH.
   -Similarly, the rest of the display funtions work to display the digits in their proper places.
   -Although at one instant only on display unit displays the digit, as this process is done at a very high frequency, it seems as if all the digits are displayed at once.


