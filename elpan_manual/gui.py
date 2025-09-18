import tkinter as tk

root = tk.Tk()
root.title("Grid Layout Example")
root.geometry("600x600")  # atur ukuran window

frame = tk.Frame(root)
frame.pack(expand=True, fill="both", padx=10, pady=10)

# Membuat label sebagai div
div1 = tk.Label(frame, text="1", bg="lightblue")
div2 = tk.Label(frame, text="2", bg="lightgreen")
div3 = tk.Label(frame, text="3", bg="lightcoral")
div4 = tk.Label(frame, text="4", bg="lightyellow")
div5 = tk.Label(frame, text="5", bg="lightpink")

# Atur row & column agar proporsional
for i in range(4):
    frame.columnconfigure(i, weight=1, minsize=100)
for i in range(6):
    frame.rowconfigure(i, weight=1, minsize=100)

# Grid layout
div1.grid(row=0, column=0, rowspan=4, columnspan=2, sticky="nsew", padx=4, pady=4)
div2.grid(row=0, column=2, rowspan=2, columnspan=2, sticky="nsew", padx=4, pady=4)
div3.grid(row=2, column=2, rowspan=2, columnspan=2, sticky="nsew", padx=4, pady=4)
div4.grid(row=4, column=0, rowspan=2, columnspan=2, sticky="nsew", padx=4, pady=4)
div5.grid(row=4, column=2, rowspan=2, columnspan=2, sticky="nsew", padx=4, pady=4)

root.mainloop()
