#######################################
# 	SwarmyHive:
#		Ilhan Delic - 0914619
#		Marco Fazari - 0887361
#		Rowalski Wever - 0918221
#		Paul Wondel - 0947421
#
#######################################

# Defining our function as seidel which takes 3 arguments 
# as A matrix, Solution and B matrix 

def seidel(a, x ,b): 
    #Finding length of a(3)     
    n = len(a)                 
    # for loop for 3 times as to calculate x, y , z 
    for j in range(0, n):         
        # temp variable d to store b[j] 
        d = b[j]                 
        
        # to calculate respective xi, yi, zi 
        for i in range(0, n):     
            if(j != i): 
                d-=a[j][i] * x[i] 
        # updating the value of our solution         
        x[j] = d / a[j][j] 
    # returning our updated solution         
    return x     

# int(input())input as number of variable to be solved                 
n = 3                            
a = []                             
b = []         
# initial solution depending on n(here n=3)                     
x = [0, 0, 0]                         
a = [[6,3,-2],[4,5,-4],[2,-3,5]]  #values are filled here
b = [6,2,11] #and here
print(x) 

#loop run for m times depending on m the error value 
for i in range(0, 50):             
    x = seidel(a, x, b) 
    #print each time the updated solution 
    print(x)

print("\n")
print("x=1/"+str(a[0][0])+"("+str(-a[0][1])+"y+"+str(-a[0][2])+"z+"+str(b[0])+")")	
print("y=1/"+str(a[1][1])+"("+str(-a[1][0])+"x+"+str(-a[1][2])+"z+"+str(b[1])+")")
print("z=1/"+str(a[2][2])+"("+str(-a[2][0])+"x+"+str(-a[2][1])+"y+"+str(b[2])+")")

