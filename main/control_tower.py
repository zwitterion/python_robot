
# https://stackoverflow.com/questions/34276663/tkinter-gui-layout-using-frames-and-grid
import time
import traceback
import multiprocessing as mp
from multiprocessing.managers import BaseManager
import threading
import logging

import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

from config import Config
from message import Message
from messagebus_manager import MessageBusManager, ProcessNames

#class MessageBusManager(BaseManager):
#    pass

class ProcessMessages (threading.Thread):
    def __init__(self, _name, config, mbus, on_state_change, on_error):
        threading.Thread.__init__(self)
        self.name = _name
        self.config = config
        self.message_bus = mbus
        self.callback = on_state_change
        self.on_error_callback = on_error
        self.daemon = True
        return

    def run(self):
        self.config.log.info("control tower message processing thread has started (%s)" % self.name )
        

        self.message_bus.subscribe(topic = "map_updates", subscriber=self.name)

        while True: 
            try:
                msg = self.message_bus.receive(self.name, latest_message=True)  # latest=True becasue we only care about latest (discard older messages)
                self.config.log.info("control tower received message %s" % msg.cmd )
                data = np.array(msg.params['grid'])
                robot_position = msg.params['robot_position']
                #robot_position[0] = int(np.round(robot_position[0]))
                #robot_position[1] = int(np.round(robot_position[1]))

                planner_path = msg.params['path']

                #print("PATH", planner_path)

                self.callback(data, robot_position, planner_path)

            except (KeyboardInterrupt, SystemExit):
                self.config.log.info("control tower: process messages Ctrl+C" )
                return # exit process

            except (EOFError, BrokenPipeError):
                self.config.log.warning("control tower: Broken Pipe.  Exiting thread" )
                self.on_error_callback()
                return # exit process

            except Exception as e:
                self.config.log.exception(e)
                #print("control tower got exception %s" % e)
                traceback.print_exc()

                pass



class Application(tk.Frame):
    def __init__(self, config, message_bus, master=None):
        super().__init__(master)
        self.master = master
        self.name = "control_tower"
        
        self.config = config
        self.message_bus = message_bus
        self.map_data =  np.ones(self.config['map_grid_size']) * 127.
        self.robot_position = [-1,-1,-1]
        self.last_position = self.robot_position
        self.target_position = [-1,-1]

        self.preview_canvas_width = 0
        self.preview_canvas_height = 0

        # reference to tk image representing the robot
        self.robot_img = None
        # references to rectangles used to show the main grid
        self.rectangles = []
        self.planner_path = []
        self.create_widgets()
        
        self.map_update_lock = threading.Lock()
        self.is_updating = False

        if (message_bus is not None):
            self.worker_thread = self.start_receiving_thread()

        

    def start_receiving_thread(self):
        self.config.log.info("starting receiving worker thread" )
        worker_thread = ProcessMessages(self.name, self.config, self.message_bus, self.on_state_change, self.on_msg_bus_error)
        worker_thread.start()
        return worker_thread

    # callback 
    def on_state_change(self, data, robot_position, planner_path):
        self.map_data = data
        self.robot_position = robot_position
        self.planner_path = planner_path
        self.show_map()

        #self.config.log.info("control tower robot position: {} path len: {}".format(self.robot_position, len(self.planner_path)))

        return 

    def on_msg_bus_error(self):
        # quit Tk mainloop
        self.master.quit()
        return

    def on_target_selected(self, event):
        
        n_rows = self.map_data.shape[0]
        n_columns = self.map_data.shape[1]

        #row = int(n_rows * event.x/self.top_frame_canvas.winfo_width())
        #col = int(n_columns * event.y/self.top_frame_canvas.winfo_height())

        x = int(n_columns * event.x/self.top_frame_canvas.winfo_width())
        y = int(n_rows * event.y/self.top_frame_canvas.winfo_height())

        # assign new target position
        self.target_position = [x, n_rows-y-1]
        self.show_map()

        # update planer...
        if (self.message_bus!=None):
            msg = Message(Message.set_target_position, self.target_position)
            self.message_bus.send("planner", msg)
            self.config.log.info("sent message: {} to: [{}]".format(msg.to_json(), "planner"))


        return

    def close(self):
        self.worker_thread.stop()
        self.worker_thread.join()

    def scale_rectangle(self, rect, scale_x, scale_y):
        rect = ( rect[0]*scale_x, rect[1]*scale_y, rect[2]*scale_x, rect[3]*scale_y)
        return rect

    def show_map(self):
        
        self.map_update_lock.acquire(True)
        if self.is_updating: 
            # update in progress - try to refresh again in 100ms
            self.map_update_lock.release()
            root.after(100, self.show_map) 
            return

        self.is_updating = True
        self.map_update_lock.release()

        canvas = self.top_frame_canvas
        
        try:
            n_rows = self.map_data.shape[0]
            n_columns = self.map_data.shape[1]

            scale_x = self.top_frame_canvas.winfo_width()/n_columns
            scale_y = self.top_frame_canvas.winfo_height()/n_rows

            # if window size changed invalidate rectangles list so they will be redrawn
            update_coords = False
            if ((self.top_frame_canvas.winfo_width() != self.preview_canvas_width) or 
                (self.top_frame_canvas.winfo_height() != self.preview_canvas_height)):
                self.preview_canvas_width = self.top_frame_canvas.winfo_width() 
                self.preview_canvas_height = self.top_frame_canvas.winfo_height()
                #self.rectangles = []
                update_coords = True
                pass

            h = 1
            w = 1

            # first pass create the rectangles, then just update clr
            create_rectangles = True if len(self.rectangles) == 0 else False
            rectangle_index = 0

            c_robot = int(np.round(self.robot_position[0]))
            r_robot = int(np.round(self.robot_position[1]))

            for r in range(0, n_rows):
                for c in range(0, n_columns):    
                    # assign one of the grayXX colors 
                    clr = "gray" + str(int(99*self.map_data[n_rows-r-1,c]/255.0))
                    # if robot is here color red
                    if (c_robot == c ) and (r_robot == n_rows-r-1):
                        # current robot position
                        clr = "red"
                    elif [c, n_rows-r-1] in self.planner_path:
                        clr = "green"

                    if create_rectangles:
                        rect = self.scale_rectangle((c,r, c+w, r+h), scale_x, scale_y)
                        new_rect = canvas.create_rectangle(rect, activefill="yellow", fill=clr)
                        self.rectangles.append(new_rect)
                    else:
                        new_rect = self.rectangles[rectangle_index]
                        canvas.itemconfig(new_rect, fill=clr)

                        if update_coords:
                            rect = self.scale_rectangle((c,r, c+w, r+h), scale_x, scale_y)
                            canvas.coords(new_rect, rect)

                        rectangle_index +=1 


            canvas.bind("<Button-1>", self.on_target_selected)

            # show target
            if self.target_position[0] >= 0:
                x  = (self.target_position[0] + 0.50) * scale_x - 10
                y  = (n_rows-1-self.target_position[1] + 0.50) * scale_y - 10
                self.show_target_icon(x,y)

            # show robot positon
            x  = int(np.round((self.robot_position[0]+0.5) * scale_x - 10.))
            y  = int(np.round((n_rows-1.0-self.robot_position[1] + 0.5) * scale_y - 10.))
            theta=self.robot_position[2]
            self.show_robot_icon(x,y,theta)

        except Exception as e:
            print("EXCEPTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", e)
            exit(0)


        finally:
            self.is_updating = False


        return


    def redraw_map(self,event):
        #print("Event", event.width, event.height)
        #print("Cfg", self.top_frame_canvas.winfo_width(), self.top_frame_canvas.winfo_height() )
        self.show_map()
        return

    def create_widgets(self):
        # command content variables
        self.command_content = tk.StringVar()
        self.command_content.set("")

        addresses = [k for k in ProcessNames.__dict__.keys() if k[0] != "_"]
        self.command_address = tk.StringVar()
        self.command_address.set(addresses[0])
        
        command_ids = [k for k in Message.__dict__.keys() if k[0] != "_"]
        self.command_ids_var = tk.StringVar()
        self.command_ids_var.set(command_ids[0])

        # create the main frames
        self.top_frame = tk.Frame(bg='cyan', padx=2, pady=2)
        self.top_frame.grid(row=0, rowspan=8, column=0, columnspan=2, sticky="wens")
        self.top_frame.grid_columnconfigure(0, weight=1) # autoresize ???
        self.top_frame.grid_rowconfigure(0, weight=1) # autoresize ???

        self.right_frame = tk.Frame(bg='red', padx=2, pady=2)
        self.right_frame.grid(row=0, rowspan=8, column=2, sticky="wens")

        self.command_line_frame = tk.Frame(bg='blue', padx=2, pady=2)
        self.command_line_frame.grid(row=9, column=0, columnspan=2, sticky="wens")
        self.command_line_frame.grid_columnconfigure(1, weight=1) # autoresize command text section

        # configure top frame (visualizer)
        self.top_frame_canvas = tk.Canvas(self.top_frame, highlightthickness=0, highlightbackground="green")
        self.top_frame_canvas.grid(row=0, column=0, sticky="wens")
        self.top_frame_canvas.grid_columnconfigure(0, weight=1) # autoresize ???
        self.top_frame_canvas.grid_rowconfigure(0, weight=1) # autoresize ???

        # raw image for robot 
        # image will be positioned in self.show_robot_icon() 
        self.robot_img_png = Image.open("./robot.png").resize((20,20), Image.ANTIALIAS)
        
        # image for target position 
        self.target_img_png = Image.open("./target.png").resize((20,20), Image.ANTIALIAS)
        self.target_img_ref = ImageTk.PhotoImage(self.target_img_png)
        self.target_img = self.top_frame_canvas.create_image(0, 0, anchor="nw", image=self.target_img_ref)
        #self.top_frame_canvas.itemconfig(self.target_img, image=self.target_img_ref)


        self.top_frame_canvas.bind('<Configure>', self.redraw_map)

        # configure right frame
        self.quit = tk.Button(self.right_frame, text="QUIT", fg="red", command=self.master.destroy)
        self.quit.grid(row=0, column=0)
        

        # configure command line frame
        self.command_address_dropdown = tk.OptionMenu(self.command_line_frame, self.command_address, *addresses)
        self.command_address_dropdown.grid(row=0, column=0, columnspan=1, sticky="we",padx=10)

        self.command_address_cmdid_dropdown = tk.OptionMenu(self.command_line_frame, self.command_ids_var, *command_ids)
        self.command_address_cmdid_dropdown.grid(row=0, column=1, columnspan=1, sticky="we",padx=10)

        self.command_text = tk.Entry(self.command_line_frame)
        self.command_text.grid(row=0, column=2, columnspan=7, sticky="we",padx=10)
        self.command_text.bind('<Key-Return>', self.send_message)
        self.command_text["textvariable"] = self.command_content

        # command send button
        self.send_button = tk.Button(self.command_line_frame)
        self.send_button.grid(row=0, column=9, sticky="we")
        self.send_button["text"] = "Send"
        self.send_button["command"] = self.send_message
        
    def show_robot_icon(self, x, y, theta):

        # only update if position has changed 
        if (self.last_position != [x,y,theta]):

            self.robot_img_ref = ImageTk.PhotoImage(self.robot_img_png.rotate(theta))
            if (self.robot_img is None):
                self.robot_img = self.top_frame_canvas.create_image(x, y, anchor="nw", image=self.robot_img_ref)
            else:
                # update existing 
                self.top_frame_canvas.itemconfig(self.robot_img, image=self.robot_img_ref)
                self.top_frame_canvas.coords(self.robot_img, (x, y))

            # update last position so we can check above if it has changed 
            self.last_position = [x,y,theta]

        # ensure image is top layer so it is visible
        self.top_frame_canvas.tag_raise(self.robot_img)

        return

    def show_target_icon(self, x, y):

        self.top_frame_canvas.coords(self.target_img, (x, y))

        # ensure image is top layer so it is visible
        self.top_frame_canvas.tag_raise(self.target_img)

        return

    


    def send_message(self, event=None):
        target = self.command_address.get()
        command_id = self.command_ids_var.get()
        command_params = self.command_text.get()
        msg = Message(Message.__dict__[command_id], command_params, reply_to = self.name)
        self.message_bus.send(target, msg)
        self.config.log.info("sent message: {} to: [{}]".format(msg.to_json(), target))
        self.command_content.set("")


if __name__ == "__main__":

    config = Config()
    mp.current_process().authkey = config['authkey']
    message_bus = None
    try:
        #MessageBusManager.register('mbus') 
        #message_bus_manager = MessageBusManager(address=config["message_bus_address_remote"], authkey=config['authkey']) # address=('foo.bar.org', 50000), authkey=b'abracadabra'
        #message_bus_manager.connect()
        #message_bus = message_bus_manager.mbus(None)
        message_bus = MessageBusManager.get_message_bus(config)

        
        config.log.info('The control tower is ready')
    except ConnectionRefusedError:
        config.log.info('Message bus is not available. Starting disconnected')


        
    root = tk.Tk()
    root.grid_rowconfigure(1, weight=1)
    root.grid_columnconfigure(0, weight=1)
    s=1.5
    root.geometry('{}x{}'.format(int(s* 560), int(s*350)))

    app = Application(master=root, config=config, message_bus = message_bus)
    app.master.title("Control Tower")
    #app.master.minsize(400, 400)

    app.mainloop()

    