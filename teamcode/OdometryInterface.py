import tkinter as tk
import os

clicknum = 0
clicknum2 = 0

def motion(event):
    x, y = event.x, event.y
    btn_text.set('{}, {}'.format(x-3, 505-y))

prevx, prevy = 0, 0
def callback11(event):
    global clicknum, panel, prevx, prevy
    clicknum += 1
    if clicknum == 1:
        prevx, prevy = event.x, event.y
        panel.create_oval(prevx - 6, prevy - 6, prevx + 6, prevy + 6, fill = "red")
    x, y = event.x, event.y
    if clicknum > 1:
        panel.create_line(prevx, prevy, x, y, width = 3)
        prevx, prevy = event.x, event.y
        if spdvar.get() != "" and timeoutvar.get() != "":
            print('To({}, {}, {}, {}, {});'.format(x-3, 505-y, spdvar.get(), timeoutvar.get(), wobble_var.get()))
    panel.create_oval(x - 6, y - 6, x + 6, y + 6, fill = "red")

def callback21(event):
    x, y = event.x, event.y
    global clicknum2, panel, prevx1, prevy1
    clicknum2 += 1
    if clicknum2 == 1:
        prevx1, prevy1 = event.x, event.y
    if clicknum2 > 1:
        panel.create_line(prevx1, prevy1, x, y, width = 3, fill = "green")
        prevx1, prevy1 = event.x, event.y
    panel.create_oval(x - 6, y - 6, x + 6, y + 6, fill = "green", outline = "black")

root = tk.Tk()
root.title('TeslaRunner')
root.geometry("1200x600")

btn_text = tk.StringVar(root)
btn_text.set("0, 0")
timeoutvar = tk.StringVar(root)
timeoutvar.set("1")
spdvar = tk.StringVar(root)
spdvar.set("1")
wobble_var = tk.BooleanVar(root)
wobble_var.set(False)

btn = tk.Button(root, width = 5, height = 2, textvariable = btn_text, command = motion)
btn.place(x = 295, y =0)

img = tk.PhotoImage(file="Поле.png")
panel = tk.Canvas(root, width = 415, height = 520, bd=0, highlightthickness=0)
panel.bind('<Motion>', motion)
panel.bind('<Button-1>', callback11)
panel.bind('<Button-3>', callback21)
panel.focus_set()
panel.create_image(0, 0, image = img, anchor = "nw")
panel.place(x = 110, y = 60)

textfield = tk.Entry(root, textvariable = timeoutvar)
textfield.place(x = 600, y = 60)
btntime = tk.Button(root, text = "Timeout(ms)", width = 20, height = 1)
btntime.place(x = 590, y = 85)

textfield2 = tk.Entry(root, textvariable = spdvar)
textfield2.place(x = 600, y = 120)
btnspd = tk.Button(root, text = "Speed", width = 20, height = 1)
btnspd.place(x = 590, y = 145)
w1 = tk.Radiobutton(text = 'Wobble false', variable = wobble_var, value = False)
w2 = tk.Radiobutton(text = 'Wobble true', variable = wobble_var, value = True)
w1.place(x = 590, y = 180)
w2.place(x = 590, y = 210)


root.mainloop()
