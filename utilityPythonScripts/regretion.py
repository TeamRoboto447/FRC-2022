
def regRaw(x,y):
    if len(x)!=len(y):
        raise ValueError("x and y should be the same length")
    n=len(x)
    b=(sum(y)*sum(X**2 for X in x)-sum(x)*sum(X*Y for X,Y in zip(x,y)))/(n*sum(X**2 for X in x)-sum(x)**2)
    m=(n*sum(X*Y for X,Y in zip(x,y))-sum(x)*sum(y))/(n*sum(X**2 for X in x)-sum(x)**2)
    return m,b

def regress(dist,xIn):
    return regRaw(xIn,dist)

if __name__=="__main__":
    print(regress([7,9,15,21],[2,3,4,5]))
