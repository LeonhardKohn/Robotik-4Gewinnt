from ultralytics import YOLO
import cv2

#Parameter: image als open-cv-image, 
# turns: im wievielten Zug befinden wir uns gerade (int), 
# colour: in welcher Farbe spielen wir (String "red" oder "blue")
def get_board(image, turns, colour):
    technically_correct = False
    semantically_correct = False
    board = [[None,None,None,None,None,None,None],
            [None,None,None,None,None,None,None],
            [None,None,None,None,None,None,None],
            [None,None,None,None,None,None,None],
            [None,None,None,None,None,None,None],
            [None,None,None,None,None,None,None]]

    #prediction see: https://docs.ultralytics.com/modes/predict/#key-features-of-predict-mode
    # Load a model
    model = YOLO("/weights/best.pt")  #todo: insert richtigen model-Pfad hier, Bildformat sollte so eigentlich passen!

    # Run batched inference on a list of images
    results = model(image, stream = True)  # return a list of Results objects, stream = True only keeps the results of the current frame or data point in memory
    #todo: hier nochmal schauen, muss gar nicht so viel processen, evtl. Bild mit Boxen ausgeben,
    # auf jeden Fall schauen, wie ich nur classprediction rausbekomme, die reicht uns ja
    # Process results list
    for result in results:
        boxes = result.boxes  # Boxes object for bounding box outputs
        masks = result.masks  # Masks object for segmentation masks outputs
        keypoints = result.keypoints  # Keypoints object for pose outputs
        probs = result.probs  # Probs object for classification outputs
        obb = result.obb  # Oriented boxes object for OBB outputs
        result.show()  # display to screen

    
    results= [[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0], 
                    [[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],
                    [[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],
                    [[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],
                    [[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],
                    [[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],[[0,0],[0,0],0],
                    ]
    amounts = [0,0,0] #each detected element is of the form [[x1,y1],[x2,y2],V], amounts sind [empty, red, blue]
    
    # output zu board
    for i in results:
        j = 0
        while j <6:
            k = 0
            while k<7:
                board[j,k] = results[i][2]  # gibt nur den zurückgegebenen Wert aus # todo: muss Wert umgerechnet werden? Dann hier!
                k += 1                          
            j += 1
        
    technically_correct = is_technically_correct(board)
    if technically_correct == False:
        board, technically_correct = fix_board_alignment(board, results)
        
    semantically_correct = is_semantically_correct(board, turns, colour,amounts)
    if semantically_correct == False:
        fix_board_alignment(board)
    
    return board, technically_correct, semantically_correct #0 =empty, 1=rot, 2=blau

#Tests if board recognition is technically correct
#If each position in the array is either 0, 1 or 2 the board is technically correct
def is_technically_correct(board):
    i = 0
    technically_correct = True
    while i<6:
        j = 0
        while j<7:
            if board[i][j] != 0 or board[i][j] != 1 or board[i][j] != 2:
                technically_correct = False   
                print("Das board wurde technisch falsch erkannt und muss korrigiert werden!")
            j += 1
        i += 1       
    
    if technically_correct:
        print("Das board wurde technisch richtig erkannt!")
    return technically_correct
    
   
#tries to fix the board alignment by checking for noticable breaks in the y-coordinate indicating a new line
#wir speichern Zeilenumbrüche als counter
def fix_board_alignment(board, model_output_list):
    technically_correct = True
    y_coordinate = model_output_list[0][0][1]
    length = model_output_list.length
    counter = []
    i = 1
    while i <length:
        if model_output_list[i][0][1] - y_coordinate >= 10:  #todo: wie viele Pixel sind hier üblich? Anpassen, wenn Bilderkennung steht!
            counter.append(i-1) #wir zählen in der Liste die letzte Position vor dem Zeilenumbruch
        
    # output zu board
    # board füllen nach Zeilenumbrüchen
    next_line_break = counter[0] 
    counter_check = 7 #was wenn der counter-Wert größer ist als die Zeilenlänge?
    for i in model_output_list:
        j = 0
        while j <6:
            k = 0
            check = next_line_break
            if next_line_break > counter_check:
                check = counter_check
            while k<check : #fülle eine Zeile auf bis zum nächsten linebreak (gefunden oder enforced)
                board[j,k] = model_output_list[i][2]  # gibt nur den zurückgegebenen Wert aus # todo: muss Wert umgerechnet werden? Dann hier!
                k += 1                          
            j += 1
            next_line_break = counter[j]
            counter_check += 7
    
    #jetzt prüfen: gibt es nones, falls ja, freischwebend? dann auf empty setzen, sonst Fehler auswerfen
    i= 0
    while i<4: #unterste Zeile müssen wir nicht auf freischwebende Steine überprüfen
        j = 0
        while j < 7: #wir gehen jede Zeile durch
            if board[i][j] == None:
                if board[i][j] != 0 and board[i+1][j] == 0: # wenn das Element nicht empty ist, aber das darunter liegende
                    board[i][j] = 0 # freischwebende nones setzen wir auf empty
                    print("Freischwebendes Element an Stelle: [" + i + ","+ j+"] entdeckt und auf empty gesetzt!")
                else: technically_correct = False
            j += 1
        i += 1
    
    if technically_correct:
        print("The board could be fixed and is correct now")
    else:
        print("An attempt was made to fix the board's technical problems, but they could not be fixed.")
    return board, technically_correct
    
def is_semantically_correct(board,turns, colour, amounts):  
    semantically_correct = True
    max_stones = turns-1 # Bsp.: wir sind in Zug 6, also liegen maximal 5 Steine
    red_stones = amounts[1]
    blue_stones = amounts[2]
    all_stones = red_stones + blue_stones
    
    #entspricht Anzahl Steine der turn-Anzahl?
    if all_stones != max_stones:
        semantically_correct = False
        print("Falsche Anzahl Steine erkannt!")
        if all_stones > max_stones:
            print("Zu viele Steine erkannt!")
        else:
            print("Zu wenige Steine erkannt!")
        return semantically_correct
    
    # entspricht die Farbaufteilung der Steine der nach der Anzahl der Züge richtigen?
    # wenn eigene Farbe beginnt, sind wir im ungeraden Zug dran und es müssen pro Farbe exakt gleich viele Steine sein
    if turns % 2 != 0 and red_stones != blue_stones:
        semantically_correct = False
        if red_stones > blue_stones:
            accusation(colour, "red")
        else:
            print("Es wurden zu viele blaue Steine erkannt!")
            accusation(colour, "blue")
    #wenn andere Farbe beginnt, sind wir im geraden Zug dran und es muss ein Stein mehr von der anderen Farbe sein
    elif  turns % 2 == 0 and red_stones != blue_stones:   
        if red_stones > blue_stones:
            accusation(colour, "red")
        else:
            print("Es wurden zu viele blaue Steine erkannt!")
            accusation(colour, "blue")
            
    # sind keine freischwebende Steine vorhanden?
    i= 0
    while i<4: #unterste Zeile müssen wir nicht auf freischwebende Steine überprüfen
        j = 0
        while j < 7: #wir gehen jede Zeile durch
            if board[i][j] != 0 and board[i+1][j] == 0: # wenn das Element nicht empty ist, aber das darunter liegende
                semantically_correct = False
                print("Freischwebendes Element an Stelle: [" + i + ","+ j+"] entdeckt!")
            j += 1
        i += 1
    
    if semantically_correct:
        print("Das board ist semantisch richtig!")
    return semantically_correct

def accusation(colour, colour_too_much):
    print("Es wurden zu viele " + colour_too_much + " Steine erkannt!")
    if colour == colour_too_much:
        print("Warum hast du keinen Zug gemacht?")
    else:
        print("SCHUMMELEI! SABOTAGE! BETRUG! DU HAST ZU VIELE STEINE GESPIELT!")
