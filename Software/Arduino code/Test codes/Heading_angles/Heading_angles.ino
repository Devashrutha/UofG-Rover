#include <iostream>
using namespace std;
#include <math.h>
#include <cmath>


int main() {

  int wheelbase = 450;
  int trackwidth = 430;
  // Defining the turning angles alpha1(a1) and alpha2(a2) and alpha_rad.
  float alpha1, alpha2, alpha_rad;
  // Defining the heading angle alpha(a).
  int alpha;
  /* for loop goes through all the angles from 0 degrees to 89 degrees
   to fing the range of heading angles that have a positive soultion for alpha1 and alpha2. */ 
  for (alpha = 1; alpha < 89; alpha++) {
    // Converting degrees to radians.
    alpha_rad = alpha * M_PI / 180;

    alpha1 = atan(wheelbase / ((wheelbase / tan(alpha_rad)) + trackwidth));

    alpha2 = atan(wheelbase / ((wheelbase / tan(alpha_rad)) - trackwidth));
    // Converting radians to degrees and printing the heading angle, a1 and a2.
    cout << alpha << " A1: " << alpha1 * 180 / 3.1415 << " A2: " << alpha2 * 180 / 3.1415 << endl;
  }

  return 0;
}
