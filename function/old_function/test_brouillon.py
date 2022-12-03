
#%%
import threading
import time


data_lock = threading.Lock()
xval = 20
data_lock.
def writing_thread():
    global xval
    while 1:
        xval += 1
        
threading.Thread(target=writing_thread,).start()

while True:
    global xval
    print(xval)
    time.sleep(1)
#%%
    
            
#def reading_thread():
#    xval2 = xval
#    print(xval2)
#    time.sleep(0.5)
            
#%%
t1 = threading.Thread(target=writing_thread,)
t2 = threading.Thread(target=reading_thread,)
# %%
t1.start()
t2.start()
# %%
