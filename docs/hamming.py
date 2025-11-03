import math
import numpy as np

if __name__ == "__main__":
    N = 15000
    NormF = 2**15
    hamming = np.sin(np.pi * np.arange(N) / N)**2 * NormF
    print(np.max(hamming))
    fout = open('hamming.c', 'w+')
    fout.write(f'''
#include "pedalinator_config.h"
#include "arm_math.h"
               
q15_t hamming_window[{N}] = {{
    {','.join(str(int(x)) for x in hamming)}
}};
''')