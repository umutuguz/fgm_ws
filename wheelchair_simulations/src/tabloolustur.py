import pandas as pd

# Örnek veri
data = """
100 tanede 32%, 125 tanede 31.2%

Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 1
Goal Reached!  Total distance traveled is: 34.9322 || Avg execution time per cycle is: 0.000593668
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 2
Collision occured!  ||  Avg execution time per cycle is: 0.000852761
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 3
Collision occured!  ||  Avg execution time per cycle is: 0.000676259
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 4
Goal Reached!  Total distance traveled is: 34.731 || Avg execution time per cycle is: 0.000657388
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 5
Goal Reached!  Total distance traveled is: 34.4271 || Avg execution time per cycle is: 0.00068559
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 6
Collision occured!  ||  Avg execution time per cycle is: 0.000752193
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 7
Collision occured!  ||  Avg execution time per cycle is: 0.000806616
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 8
Goal Reached!  Total distance traveled is: 34.7333 || Avg execution time per cycle is: 0.000655941
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 9
Goal Reached!  Total distance traveled is: 34.9348 || Avg execution time per cycle is: 0.000681239
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 10
Goal Reached!  Total distance traveled is: 34.7381 || Avg execution time per cycle is: 0.000602632
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 11
Collision occured!  ||  Avg execution time per cycle is: 0.000889655
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 12
Collision occured!  ||  Avg execution time per cycle is: 0.000794595
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 13
Goal Reached!  Total distance traveled is: 34.6391 || Avg execution time per cycle is: 0.000668269
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 14
Collision occured!  ||  Avg execution time per cycle is: 0.000774936
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 15
Collision occured!  ||  Avg execution time per cycle is: 0.000824138
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 16
Collision occured!  ||  Avg execution time per cycle is: 0.000857988
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 17
Collision occured!  ||  Avg execution time per cycle is: 0.000867987
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 18
Collision occured!  ||  Avg execution time per cycle is: 0.000741085
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 19
Collision occured!  ||  Avg execution time per cycle is: 0.000798137
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 20
Goal Reached!  Total distance traveled is: 34.8379 || Avg execution time per cycle is: 0.000640097
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 21
Collision occured!  ||  Avg execution time per cycle is: 0.000792453
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 22
Collision occured!  ||  Avg execution time per cycle is: 0.000698551
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 23
Collision occured!  ||  Avg execution time per cycle is: 0.000801688
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 24
Collision occured!  ||  Avg execution time per cycle is: 0.000689524
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 25
Goal Reached!  Total distance traveled is: 34.1256 || Avg execution time per cycle is: 0.000683297
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 26
Collision occured!  ||  Avg execution time per cycle is: 0.000885106
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 27
Collision occured!  ||  Avg execution time per cycle is: 0.000724638
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 28
Goal Reached!  Total distance traveled is: 34.73 || Avg execution time per cycle is: 0.00070098
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 29
Collision occured!  ||  Avg execution time per cycle is: 0.000722646
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 30
Goal Reached!  Total distance traveled is: 34.7561 || Avg execution time per cycle is: 0.00065544
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 31
Collision occured!  ||  Avg execution time per cycle is: 0.000810345
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 32
Collision occured!  ||  Avg execution time per cycle is: 0.000789474
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 33
Collision occured!  ||  Avg execution time per cycle is: 0.000735192
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 34
Collision occured!  ||  Avg execution time per cycle is: 0.00073814
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 35
Collision occured!  ||  Avg execution time per cycle is: 0.000806604
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 36
Collision occured!  ||  Avg execution time per cycle is: 0.000699153
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 37
Goal Reached!  Total distance traveled is: 35.1378 || Avg execution time per cycle is: 0.000679121
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 38
Collision occured!  ||  Avg execution time per cycle is: 0.00085
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 39
Collision occured!  ||  Avg execution time per cycle is: 0.000697368
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 40
Goal Reached!  Total distance traveled is: 34.3514 || Avg execution time per cycle is: 0.000673913
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 41
Collision occured!  ||  Avg execution time per cycle is: 0.000847716
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 42
Collision occured!  ||  Avg execution time per cycle is: 0.000830303
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 43
Collision occured!  ||  Avg execution time per cycle is: 0.000793269
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 44
Collision occured!  ||  Avg execution time per cycle is: 0.000815642
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 45
Collision occured!  ||  Avg execution time per cycle is: 0.000779476
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 46
Collision occured!  ||  Avg execution time per cycle is: 0.000831522
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 47
Collision occured!  ||  Avg execution time per cycle is: 0.00106061
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 48
Collision occured!  ||  Avg execution time per cycle is: 0.000777778
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 49
Goal Reached!  Total distance traveled is: 34.9607 || Avg execution time per cycle is: 0.000720648
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 50
Collision occured!  ||  Avg execution time per cycle is: 0.000843866
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 51
Goal Reached!  Total distance traveled is: 34.4849 || Avg execution time per cycle is: 0.00074734
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 52
Collision occured!  ||  Avg execution time per cycle is: 0.00086087
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 53
Goal Reached!  Total distance traveled is: 34.4552 || Avg execution time per cycle is: 0.000675299
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 54
Collision occured!  ||  Avg execution time per cycle is: 0.000726384
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 55
Collision occured!  ||  Avg execution time per cycle is: 0.000834171
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 56
Collision occured!  ||  Avg execution time per cycle is: 0.000757353
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 57
Collision occured!  ||  Avg execution time per cycle is: 0.000680435
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 58
Collision occured!  ||  Avg execution time per cycle is: 0.000744635
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 59
Goal Reached!  Total distance traveled is: 35.2301 || Avg execution time per cycle is: 0.000666667
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 60
Collision occured!  ||  Avg execution time per cycle is: 0.00068552
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 61
Goal Reached!  Total distance traveled is: 34.9568 || Avg execution time per cycle is: 0.000636364
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 62
Collision occured!  ||  Avg execution time per cycle is: 0.000758065
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 63
Collision occured!  ||  Avg execution time per cycle is: 0.00083391
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 64
Collision occured!  ||  Avg execution time per cycle is: 0.000763689
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 65
Goal Reached!  Total distance traveled is: 35.5643 || Avg execution time per cycle is: 0.000790091
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 66
Collision occured!  ||  Avg execution time per cycle is: 0.000993902
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 67
Collision occured!  ||  Avg execution time per cycle is: 0.000929936
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 68
Goal Reached!  Total distance traveled is: 34.6411 || Avg execution time per cycle is: 0.000694915
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 69
Collision occured!  ||  Avg execution time per cycle is: 0.000857143
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 70
Goal Reached!  Total distance traveled is: 34.8437 || Avg execution time per cycle is: 0.000664368
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 71
Collision occured!  ||  Avg execution time per cycle is: 0.000881356
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 72
Goal Reached!  Total distance traveled is: 34.2114 || Avg execution time per cycle is: 0.000674419
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 73
Collision occured!  ||  Avg execution time per cycle is: 0.000803226
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 74
Goal Reached!  Total distance traveled is: 34.6928 || Avg execution time per cycle is: 0.000673387
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 75
Collision occured!  ||  Avg execution time per cycle is: 0.00103306
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 76
Collision occured!  ||  Avg execution time per cycle is: 0.000843434
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 77
Collision occured!  ||  Avg execution time per cycle is: 0.000736181
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 78
Goal Reached!  Total distance traveled is: 35.022 || Avg execution time per cycle is: 0.000620553
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 79
Collision occured!  ||  Avg execution time per cycle is: 0.000694444
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 80
Collision occured!  ||  Avg execution time per cycle is: 0.000674221
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 81
Collision occured!  ||  Avg execution time per cycle is: 0.000814815
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 82
Collision occured!  ||  Avg execution time per cycle is: 0.000729927
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 83
Collision occured!  ||  Avg execution time per cycle is: 0.000866359
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 84
Goal Reached!  Total distance traveled is: 34.8268 || Avg execution time per cycle is: 0.000744526
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 85
Collision occured!  ||  Avg execution time per cycle is: 0.000782979
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 86
Collision occured!  ||  Avg execution time per cycle is: 0.000845238
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 87
Goal Reached!  Total distance traveled is: 35.1203 || Avg execution time per cycle is: 0.000662921
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 88
Collision occured!  ||  Avg execution time per cycle is: 0.000869258
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 89
Goal Reached!  Total distance traveled is: 34.3296 || Avg execution time per cycle is: 0.000751468
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 90
Goal Reached!  Total distance traveled is: 34.8781 || Avg execution time per cycle is: 0.000651596
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 91
Collision occured!  ||  Avg execution time per cycle is: 0.000758706
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 92
Collision occured!  ||  Avg execution time per cycle is: 0.000720768
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 93
Goal Reached!  Total distance traveled is: 34.6192 || Avg execution time per cycle is: 0.00074902
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 94
Collision occured!  ||  Avg execution time per cycle is: 0.000712
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 95
Collision occured!  ||  Avg execution time per cycle is: 0.000868421
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 96
Goal Reached!  Total distance traveled is: 34.3349 || Avg execution time per cycle is: 0.000670185
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 97
Collision occured!  ||  Avg execution time per cycle is: 0.000837963
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 98
Collision occured!  ||  Avg execution time per cycle is: 0.000687273
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 99
Goal Reached!  Total distance traveled is: 34.3741 || Avg execution time per cycle is: 0.000616253
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 100
Goal Reached!  Total distance traveled is: 34.4972 || Avg execution time per cycle is: 0.0007
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 101
Goal Reached!  Total distance traveled is: 34.8193 || Avg execution time per cycle is: 0.000578022
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 102
Collision occured!  ||  Avg execution time per cycle is: 0.000903915
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 103
Collision occured!  ||  Avg execution time per cycle is: 0.000859016
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 104
Collision occured!  ||  Avg execution time per cycle is: 0.000745257
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 105
Collision occured!  ||  Avg execution time per cycle is: 0.000758242
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 106
Goal Reached!  Total distance traveled is: 34.7496 || Avg execution time per cycle is: 0.000650485
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 107
Collision occured!  ||  Avg execution time per cycle is: 0.000535865
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 108
Collision occured!  ||  Avg execution time per cycle is: 0.000815603
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 109
Collision occured!  ||  Avg execution time per cycle is: 0.000719481
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 110
Collision occured!  ||  Avg execution time per cycle is: 0.000720621
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 111
Collision occured!  ||  Avg execution time per cycle is: 0.000868687
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 112
Collision occured!  ||  Avg execution time per cycle is: 0.000695652
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 113
Goal Reached!  Total distance traveled is: 34.9071 || Avg execution time per cycle is: 0.000680688
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 114
Collision occured!  ||  Avg execution time per cycle is: 0.000965714
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 115
Goal Reached!  Total distance traveled is: 34.489 || Avg execution time per cycle is: 0.000690667
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 116
Collision occured!  ||  Avg execution time per cycle is: 0.000657061
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 117
Goal Reached!  Total distance traveled is: 35.4466 || Avg execution time per cycle is: 0.000671202
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 118
Collision occured!  ||  Avg execution time per cycle is: 0.00086859
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 119
Goal Reached!  Total distance traveled is: 34.8587 || Avg execution time per cycle is: 0.00073991
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 120
Collision occured!  ||  Avg execution time per cycle is: 0.00079927
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 121
Collision occured!  ||  Avg execution time per cycle is: 0.000828025
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 122
Collision occured!  ||  Avg execution time per cycle is: 0.000785408
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 123
Collision occured!  ||  Avg execution time per cycle is: 0.00077305
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 124
Collision occured!  ||  Avg execution time per cycle is: 0.00075
Simulation Started!  ||  Goal successfully published at (-22.0, 22.0) in world 125
Goal Reached!  Total distance traveled is: 35.0373 || Avg execution time per cycle is: 0.000688525
"""

# Satırları ve sütunları işle
lines = data.strip().split("\n")
rows = []
world_number = None
for line in lines:
    if "Simulation Started!" in line:
        world_number = int(line.split("world ")[1])
    elif "Goal Reached!" in line or "Collision occured!" in line:
        result = "Goal Reached" if "Goal Reached!" in line else "Collision"
        total_distance = float(line.split("Total distance traveled is: ")[1].split(" ||")[0]) if "Goal Reached!" in line else None
        avg_time = float(line.split("Avg execution time per cycle is: ")[1])
        rows.append([world_number, result, total_distance, avg_time])

# DataFrame oluştur ve Excel'e yaz
df = pd.DataFrame(rows, columns=["Dünya Numarası", "Sonuç", "Toplam Mesafe", "Ortalama Döngü Süresi"])
df.to_excel("simulation_results.xlsx", index=False, engine='openpyxl')
