# DEFINE TUI THINGS
import curses, time
from curses.textpad import Textbox, rectangle
from curses import wrapper
from os import system
global stdscr
stdscr = curses.initscr()


def draw(screen, x, y, text):
    # inputs:
    # screen  -  curses screen object
    # x - from 0 to 100% of the width of the screen
    # y - from 0 to 100% of the height of the screen

    if x > 100:
        x = 100
    if y > 100:
        y = 100 
    (max_y, max_x) = screen.getmaxyx()
    #print(f"max y = {max_y} max_x = {max_x}")

    coord_x = min(max(0, int(max_x * x // 100) - 1), max_x - 2)
    coord_y = min(max(0, int(max_y * y // 100) - 1), max_y - 1)
    coord_y = abs(max_y - coord_y)
    coord_y = min(max(0, max_y - (int(max_y * y // 100) - 1)), max_y - 1)


    screen.addstr(coord_y, coord_x, text, curses.color_pair(3))

def draw_tennis_ball(screen, x, y):
    # inputs:
    # screen  -  curses screen object
    # x - from 0 to 100% of the width of the screen
    # y - from 0 to 100% of the height of the screen

    if x > 100:
        x = 100
    if y > 100:
        y = 100 
    (max_y, max_x) = screen.getmaxyx()
    #print(f"max y = {max_y} max_x = {max_x}")

    coord_x = min(max(0, int(max_x * x // 100) - 1), max_x - 2)
    coord_y = min(max(0, int(max_y * y // 100) - 1), max_y - 1)
    coord_y = abs(max_y - coord_y)
    coord_y = min(max(0, max_y - (int(max_y * y // 100) - 1)), max_y - 1)
    screen.addstr(coord_y, coord_x, "O", curses.color_pair(3))

def draw_box(screen, x, y):
    # inputs:
    # screen - curses screen object
    # x - from 0 to 100% of the width of the screen
    # y - from 0 to 100% of the height of the screen

    if x > 100:
        x = 100
    if y > 100:
        y = 100
    (max_y,max_x) = screen.getmaxyx()
    coord_x = min(max(0, int(max_x * x // 100) - 1), max_x - 2)
    coord_y = min(max(0, int(max_y * y // 100) - 1), max_y - 1)
    coord_y = abs(max_y - coord_y)
    coord_y = min(max(0, max_y - (int(max_y * y // 100) - 1)), max_y - 1)

    screen.addstr(coord_y,coord_x, "B", curses.color_pair(3))


def draw_robot(screen, x, y):
    # inputs:
    # screen  -  curses screen object
    # x - from 0 to 100% of the width of the screen
    # y - from 0 to 100% of the height of the screen

    if x > 100:
        x = 100
    if y > 100:
        y = 100 
    (max_y, max_x) = screen.getmaxyx()
    #print(f"max y = {max_y} max_x = {max_x}")

    coord_x = min(max(0, int(max_x * x // 100) - 1), max_x - 2)
    coord_y = min(max(0, int(max_y * y // 100) - 1), max_y - 1)
    coord_y = abs(max_y - coord_y)
    coord_y = min(max(0, max_y - (int(max_y * y // 100) - 1)), max_y - 1)


    screen.addstr(coord_y, coord_x, "R", curses.color_pair(3)) 
def update_robot_coords(screen, robot):

    x = robot.x
    y = robot.y
    theta = robot.th

    screen.addstr(7, 1, f"x: {x}") 
    screen.addstr(8, 1, f"y: {y} ") 
    screen.addstr(9, 1, f"theta: {theta}") 


def screen_init():
    #myscreen = curses.initscr()
    
    if curses.has_colors():
        curses.start_color()
        bgColors = [curses.COLOR_BLUE, curses.COLOR_CYAN, curses.COLOR_GREEN, curses.COLOR_MAGENTA, curses.COLOR_RED, curses.COLOR_YELLOW]
        curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLUE)
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_WHITE)
        curses.init_pair(4, curses.COLOR_GREEN, curses.COLOR_WHITE)
    
    
    
    window1 = []
    window2 = []
    
    window1_width = 25 
    window1_height = curses.LINES
    
    
    window1 = [0,0, window1_width, window1_height]
    window2_width = curses.COLS - window1_width
    window2_height =  window1_height
    window2 = [window1_width, 0, window2_width, window2_height]
    
    window1obj = curses.newwin(window1[3], window1[2], window1[1], window1[0])
    window1obj.bkgd(' ', curses.color_pair(1))
    window1obj.addstr(1, 1, "Welcome!") 
    window1obj.addstr(4, 1, "Press 'q' to quit!") 
    window1obj.addstr(6, 1, "Robot Coordinates:") 
    window1obj.addstr(7, 1, "x: N/A ") 
    window1obj.addstr(8, 1, "y; N/A ") 
    window1obj.addstr(9, 1, "theta: N/A") 
    window1obj.addstr(11, 1, "Current function running:") 
    window1obj.addstr(12, 1, "N/A") 
    window1obj.addstr(15, 1, "Legend") 
    window1obj.addstr(16, 1, "'R' - Robot") 
    window1obj.addstr(17, 1, "'O' - Tennis Ball") 
    window1obj.addstr(18, 1, "'B' - Box")
    window1obj.addstr(20, 1, f"width: {curses.COLS}") 
    window1obj.addstr(21, 1, f"Height: {curses.LINES}") 
    window1obj.border(0)
    window1obj.refresh() 
    
    window2obj = curses.newwin(window2[3], window2[2], window2[1], window2[0])
    window2obj.bkgd(' ', curses.color_pair(2))
    window2obj.border(0)
    window2obj.refresh() 

    curses.noecho()
    curses.cbreak()
    curses.curs_set(False)

    time.sleep(0.1)

    return window1obj, window2obj


def read_char_in(screen):
    while 1:
        c = screen.getch()
        if c!= -1 and c == ord('q'):
           quit_tui()
           robot.stop_motors(ser)
           sys.exit()
           os._exit(1)
           return  # Exit the while loop

def quit_tui():
    #screen_init()
    # End of Program...
    # Turn off cbreak mode...
    curses.nocbreak()

    # Turn echo back on.
    curses.echo()

    # Restore cursor blinking.
    curses.curs_set(True)

    # Turn off the keypad...
    # stdscr.keypad(False)

    # Restore Terminal to original state.
    curses.endwin()

    # Display Errors if any happened:

    curses.noecho()
    curses.endwin()

def display_current_function(screen, function):
    screen.addstr(12, 1, function) 
