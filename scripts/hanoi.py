#! /usr/bin/env python3
# -*- coding: utf-8 -*-

def Solve3Hanoi(part_name,tower_name,org,tmp,dst):
    solution = []
    hanoi = [[],[],[]]
    for i in reversed(part_name):
        hanoi[0].append(i)
    
    MoveHanoi(hanoi,tower_name,solution,3,org,tmp,dst)
    
    return solution



def MoveHanoi(hn,tn,sol,n,org,tmp,dst):
    if n == 1:
        moved = hn[org].pop()
        hn[dst].append(moved)

        sol.append((moved,tn[dst]))
        return
    else:
        MoveHanoi(hn,tn,sol,n-1,org,dst,tmp)
        MoveHanoi(hn,tn,sol,1,org,tmp,dst)
        MoveHanoi(hn,tn,sol,n-1,tmp,org,dst)


if __name__ == '__main__':
    pn = ["S","M","L"]
    tn = ["A","B","C"]
    for mov in Solve3Hanoi(pn,tn,0,1,2):
        print(mov)