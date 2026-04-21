import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image
from tkinterdnd2 import DND_FILES, TkinterDnD

class ImageMerger:
    def __init__(self, root):
        self.root = root
        self.root.title("Image Merger (Drag & Drop)")
        self.root.geometry("500x300")

        self.image_paths = []

        # 拖拽区域
        self.label = tk.Label(root, text="拖拽图片到这里\n(支持多张)", 
                              relief="ridge", width=40, height=10)
        self.label.pack(pady=20)

        self.label.drop_target_register(DND_FILES)
        self.label.dnd_bind('<<Drop>>', self.drop_files)

        # 按钮
        tk.Button(root, text="选择图片", command=self.select_files).pack(pady=5)
        tk.Button(root, text="开始拼接", command=self.merge_images).pack(pady=5)

    def drop_files(self, event):
        files = self.root.tk.splitlist(event.data)
        self.image_paths.extend(files)
        messagebox.showinfo("提示", f"已添加 {len(files)} 张图片")

    def select_files(self):
        files = filedialog.askopenfilenames(filetypes=[("Images", "*.png;*.jpg;*.jpeg")])
        self.image_paths.extend(files)

    def merge_images(self):
        if len(self.image_paths) < 2:
            messagebox.showerror("错误", "至少需要两张图片")
            return

        images = [Image.open(p) for p in self.image_paths]

        # 统一高度
        min_height = min(img.height for img in images)
        resized = []

        for img in images:
            new_width = int(img.width * (min_height / img.height))
            resized.append(img.resize((new_width, min_height)))

        # 拼接
        total_width = sum(img.width for img in resized)
        new_img = Image.new("RGB", (total_width, min_height))

        x_offset = 0
        for img in resized:
            new_img.paste(img, (x_offset, 0))
            x_offset += img.width

        # 保存
        save_path = filedialog.asksaveasfilename(defaultextension=".png")
        if save_path:
            new_img.save(save_path)
            messagebox.showinfo("完成", "拼接成功！")

        self.image_paths = []  # 清空

if __name__ == "__main__":
    root = TkinterDnD.Tk()
    app = ImageMerger(root)
    root.mainloop()