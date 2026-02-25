import tkinter as tk

from axis_controller import AxisController


def main():
    root = tk.Tk()
    AxisController(root)
    root.mainloop()


if __name__ == "__main__":
    main()
