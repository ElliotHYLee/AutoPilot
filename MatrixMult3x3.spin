CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000
OBJ
  fds: "FullDuplexSerial"

pub main |i,j, a[9], b[9],c[9], d[9], inverse[9]

  fds.quickStart

  repeat i from 0 to 8
    a[i] := i
    b[i] := a[i] + 3

  subOp(@a,@b,@c)
  repeat
    waitcnt(cnt + clkfreq/10)
    fds.clear
    fds.str(String("A:"))
    fds.newline
    repeat i from 1 to 9
      fds.dec(a[i-1])
      fds.str(string(" "))
      if (i//3 == 0)
        fds.newline
    fds.newline
    fds.str(String("B:"))
    fds.newline
    repeat i from 1 to 9
      fds.dec(b[i-1])
      fds.str(string(" "))
      if (i//3 == 0)
        fds.newline

    fds.newline    
    fds.str(String("C:"))
    fds.newline
    repeat i from 1 to 9
      fds.dec(c[i-1])
      fds.str(string(" "))
      if (i//3 == 0)
        fds.newline
    d[0] := 3
    d[1] := 55
    d[2] := 1
    d[3] := 2
    d[4] := -10
    d[5] := 1
    d[6] := 0
    d[7] := 10
    d[8] := 0
    
    invOp(@d, @inverse)
    fds.dec(detOp(@d))

    fds.newline
    fds.str(String("inv:"))
    fds.newline
    repeat i from 1 to 9
      fds.dec(inverse[i-1])
      fds.str(string(" "))
      if (i//3 == 0)
        fds.newline    
    fds.newline   
     
PUB multOp(matAPtr, matBPtr, matCPtr)| i,j, rowCheck, colCheck

  rowCheck := 0
  colCheck := 0

  i:= 0
  j:= 0
  
  repeat 3
    repeat 3  
      repeat 3
        long[matCPtr][i] += long[matAPtr][rowCheck+j]*long[matBPtr][colCheck + 3*j] 
        j++
      i++
      j:=0        
      colCheck++  
    rowCheck+=3 
    colCheck :=0
            
  return
  
PUB addOp(matAPtr, matBPtr, matCPtr) | i

  i := 0
  repeat i from 0 to 8
    long[matCPtr][i] := long[matAPtr][i] + long[matBPtr][i]
  return


PUB subOp(matAPtr, matBPtr, matCPtr) | i

  i := 0
  repeat i from 0 to 8
    long[matCPtr][i] := long[matAPtr][i] - long[matBPtr][i]
  return

PUB transposeOp(matAPtr, matResultPtr) | i, j,k

  i :=0
  j :=0
  k :=0

  repeat i from 0 to 8
    long[matResultPtr][i] := long[matAPtr][3*k + j]
    k++
    if ((i+1)//3 ==0)
      j++
      k :=0
  return

PUB detOp(matAPtr) : det

  det := long[matAPtr][0]*(long[matAPtr][4]*long[matAPtr][8]-long[matAPtr][7]*long[matAPtr][5])-long[matAPtr][1]*(long[matAPtr][3]*long[matAPtr][8]-long[matAPtr][6]*long[matAPtr][5]) + long[matAPtr][2]*(long[matAPtr][3]*long[matAPtr][7]-long[matAPtr][6]*long[matAPtr][4])

PUB absolute(value)

  if (value<0)
    return -value
  else
    return value

PUB invOp(matAPtr, matResultPtr)| det, i

  det := detOp(matAPtr)
  if (det ==0)
    return -1

  long[matResultPtr][0] := long[matAPtr][4]*long[matAPtr][8]-long[matAPtr][5]*long[matAPtr][7]
  long[matResultPtr][1] := long[matAPtr][2]*long[matAPtr][7]-long[matAPtr][1]*long[matAPtr][8] 
  long[matResultPtr][2] := long[matAPtr][1]*long[matAPtr][5]-long[matAPtr][2]*long[matAPtr][4] 
  
  long[matResultPtr][3] := long[matAPtr][5]*long[matAPtr][6]-long[matAPtr][3]*long[matAPtr][8]
  long[matResultPtr][4] := long[matAPtr][0]*long[matAPtr][8]-long[matAPtr][2]*long[matAPtr][6]
  long[matResultPtr][5] := long[matAPtr][2]*long[matAPtr][3]-long[matAPtr][0]*long[matAPtr][5]
  
  long[matResultPtr][6] := long[matAPtr][3]*long[matAPtr][7]-long[matAPtr][4]*long[matAPtr][6]
  long[matResultPtr][7] := long[matAPtr][1]*long[matAPtr][6]-long[matAPtr][0]*long[matAPtr][7]
  long[matResultPtr][8] := long[matAPtr][0]*long[matAPtr][4]-long[matAPtr][1]*long[matAPtr][3]

  repeat i from 0 to 8 'rounding up for final result
    if long[matResultPtr][i] > 0
      long[matResultPtr][i] := (long[matResultPtr][i] + absolute(det)/2) / det
    else
      long[matResultPtr][i] := (long[matResultPtr][i] - absolute(det)/2) /det   