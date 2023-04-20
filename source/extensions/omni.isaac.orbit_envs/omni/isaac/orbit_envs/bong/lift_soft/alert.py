from tkinter import Tk, Button, LEFT


def bong_alert():
    tk = Tk()
    tk.title('Bong Alert')

    button = Button(tk, text='Learning Done', bg='red', font=60, width=100, height=50, command=tk.destroy)
    button.pack(side=LEFT, padx=10, pady=10)

    tk.after(3 * 1000, tk.destroy)
    tk.mainloop()


if __name__ == "__main__":
    bong_alert()