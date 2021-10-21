import pandas as pd
import os
import sys


path = "./output_new_terrains/"
evolver = sys.argv[1]
pop_size = int(sys.argv[2])
for file in os.listdir(path):
    if "all" not in file or evolver not in file:
         continue
    data = pd.read_csv(os.path.join(path, file), sep=";")
    data.reset_index(inplace=True)
    data["event→iterations"] = data["index"].apply(lambda elem: elem // pop_size)
    data = data[(data["event→iterations"] % 10 == 0) | (data["event→iterations"] == data["event→iterations"].max())]
    data.set_index("index", drop=True, inplace=True)
    data.to_csv(os.path.join(path, file), sep=";", index=False)

