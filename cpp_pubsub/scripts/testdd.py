
import time
from collections import Counter
'''
find max. value can get
'''
ttsd = [[2, 0], [4, 0], [4, 1], [7, 0], [2, 3],
        [1, 3]]  # [value,no. of non-overlap data]
ans = [0]*(len(ttsd)+1)
print(len(ttsd))

'''
i-Compute-Opt
ans[0]=0
for j=1,2,...,n do
    ans[j]=max(valuej+ans[p(j)],ans[j-1])
'''
# bottom-up approach
start_time = time.time()
for i in range(len(ttsd)+1):

    for j in range(1, i+1):
        ans[j] = max(ttsd[j-1][0]+ans[ttsd[j-1][1]], ans[j-1])
    print(ans)
end_time = time.time()
print(max(ans))
execution_time = start_time - end_time
print(execution_time)
asd=Counter().items()       
'''

'''
# top-down approach


