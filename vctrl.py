def delays(count):
    i=count
    low=10
    medium=20
    high=30
    #super_high=40
    delay=0
    print("Number of vehicles= "+ str(i))
    if i < low:
        if i<1:
            print('green light delay = 0 sec')
        elif i<4:
            delay=i*1.5
            print('green light delay = '+str(delay)+' sec')
        elif i<7:
            delay=5*1.5
            print('green light delay = '+str(delay)+' sec')
        else:
            delay=7*1.5
            print('green light delay = '+str(delay)+' sec')
    elif i < medium:
        if i<medium/2:
            delay=i*1.5
            print('green light delay = '+str(delay)+' sec')
        else:
            delay=i*1.5
            print('green light delay = '+str(delay)+' sec')
    elif i < high:
        if i<high/2:
            delay=i*1.5
            print('green light delay = '+str(delay)+' sec')
        else:
            delay=60
            print('green light delay = '+str(delay)+' sec')
    return(delay)