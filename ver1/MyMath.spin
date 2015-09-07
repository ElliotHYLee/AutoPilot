CON
  _clkmode = xtal1 + pll16x                                                    
  _xinfreq = 5_000_000

ObJ

  usb : "FullDuplexSerial"

PUB main 

  usb.quickStart

  repeat
    
    usb.decLn(sqrt(1234))


PUB sqrt(value)| x, i

  x := value

  repeat i from 0 to 20
    x := (value/x + x) /2

  return x


PUB getSign(value)

  if value >= 0
    result := 1
  else
    result := -1

PUB getAbs(value)
  if value > 0
    result := value
  else
    result := -value


' 3x3 Matrix Operations
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
      long[matResultPtr][i] := (long[matResultPtr][i] + getAbs(det)/2) / det
    else
      long[matResultPtr][i] := (long[matResultPtr][i] - getAbs(det)/2) /det   