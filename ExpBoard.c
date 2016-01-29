#include "ExpBoard.h"

int32_t getMedianValue(int32_t array[5])
{
  int32_t sortedArray[5];

  for(int i = 0; i < 5; i++)
    {
      sortedArray[i] = array[i];
    }

  bubble_sort(sortedArray, 5);

  return sortedArray[2];
}

void bubble_sort(int32_t list[], int32_t n)
{
  int c, d, t;

  for (c = 0 ; c < ( n - 1 ); c++)
    {
      for (d = 0 ; d < n - c - 1; d++)
        {
          if (list[d] > list[d+1])
            {
              /* Swapping */

              t         = list[d];
              list[d]   = list[d+1];
              list[d+1] = t;
            }
        }
    }
}
