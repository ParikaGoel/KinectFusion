
with open("depth1.txt") as f:
    txt = f.read()

print(txt.split("\n")[0])
print(len(txt.split("\n")[1].split(" ")))
