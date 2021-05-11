#!/usr/bin/python
import heapq





#funkcija za stvaranje  grafa u strukturu 'nested' dictionary
def make_graph(data):

    graph = {}
    

    for line in data:
        #odvoji i spremi u listu
        state = line.strip().split(' ')
        #prvi clan liste je roditelj
        parent = state[0]
        #odstrani ':' ispred roditeljskog cvora
        parent = parent.replace(':','')

        #obradi ostale iduce cvorove(i tezine puta do njih) koji su povezani na roditeljski cvor
        for x in state[1:]:
            child, cost = x.strip().split(',') #spremi pojedini cvor i cijenu prijelaza u zasebne varijable
            cost = int(cost)
            
            #ako nije u grafu otvori rijecnik za taj roditeljski cvor koji sadrzi kljuc sa stanjem child i vrijednost kljuca 
            #koja je cijena prijelaza izmedu roditelja i dijeteta
            if parent not in graph:
                graph[parent] = {child: cost}
                
            #ako vec roditelj postoji u grafu preskace gornji if i svejedno upisuje dijete i cijenu u rijecnik za taj roditeljski cvor 
            else:
                graph[parent][child] = cost
                
           
                

    return graph

#funkcija za ucitavanje podataka heuristike u pogodnu strukturu
def load_heuristics():
    heuristics = {}

    with open('istra_pessimistic_heuristic.txt') as f:

        lines = f.readlines() #citamo sve linije u fileu, te se svaka linija sprema u clan liste 'lines'
        
        #k
        for i in lines:
            line = i.strip().split(' ') #spremamo u drugu listu pojedino stanje i vrijednost njegove heuristike
            line[0] = line[0].replace(':','') #makivamo ':' ispred stanja
            heuristics[line[0]] = line[1]   #u rijecnik heuristike pod odredeno stanje spremamo odgovarajucu vrijednost heuristike
    
    return heuristics
            



#funkcija za prosirivanje trenutnog cvora za slucaj UCS
def expand_cost(n,next):
    
    c_n, s_n, p_n = n   #cijena,stanje,roditelj od trenutnog cvora
    s_succ, c_succ = next #stanje i cijena od iduceg cvora
    cumulative_cost = c_n + c_succ  #ukupna akumulirana cijena 

    return cumulative_cost,s_succ,s_n  #vrati: ukupnu cijenu, succ(s),s(n)




#funkcija za rekonstrukciju puta rjesenja(parents je rijecnik pa roditeljima pojedinih stanja pristupamo preko samih stanja)
def reconstruct(goal,parents):
    
    #lista u koju spremamo stanja koje smo prosli do cilja
    path = [goal]
    #vrti while petlju dok god zadnji element u listi nije jednak None (roditelj od pocetnog stanja)
    while path[-1] != None:
        path.append(parents[path[-1]])  #dodaj na kraj liste puta zadnjeg elementa sa liste roditelja, idemo u natrag od ciljnog rjesenja
        

    path.remove(None)   #ukloni roditelja od pocetnog stanja jer on zapravo nema roditelja
    path.reverse()      #obrni listu da krece od pocetnog prema zadnjem stanju

    return path



#algoritam za pretrazivanje u sirinu
def BreadthFirstSearch(s0,goal,graf):
    
    
    open = [(s0,None)] #lista otvorenih cvorova -> (stanje,roditelj)
    visited = set()   #Set skup podataka u koji pohranjujem posjecena stanja
    parents = {}    #rijecncik u koji spremam roditelje za posjecena stanja
    
    
    while open:

        n = open.pop(0) #skini prvi element sa liste 'open'
        s,p = n

        #ako stanje vec nije medu posjecenim stanjima
        if s not in visited:
            visited.add(s) #dodaj stanje pod posjeceno
            parents[s] = p #dodaj roditelja tog stanja u rijecnik
        
        #koje god ciljno stanje dostignemo prvo, vrati to stanje
        for i in goal:
            if i == s:
                return reconstruct(i,parents),len(visited) #vrati put od pocetnog do ciljnog stanja i broj posjecenih stanja

        #iteriraj po grafu
        for item in graf[s].items():
            if item[0] not in visited: #provjeri je li stanje medu vec posjecenim stanjima 
                s_next, c_next = item 
                m = (s_next,s) #novo stanje se sastoji od stanja i roditelja 
                open.append(m) #dodaj novo stanje na kraj liste 'open'
                    
    return fail

#algoritam za pretrazivanje po jednolikoj cijeni      
def UniformCostSearch(s0,goal,graf):

    
    parents = {}
    visited = set()
    openH = [] #koristim openH kao minHeap kako nebi morali sortirati listu svaki put kod dodavanja novih elemenata

    

    #dodaj u heap tuple(prava_cijena_puta,stanje,roditelj), kod stavljanja na heap sortirat ce tupleove u raspored minHeapa
    #po prvom elementu u tupleu, znaci po cijeni dosadasnjeg puta, tako da cemo uvijek na pocetku Heapa imati element
    #s najmanjom cijenom sto i zelimo
    openH.append((0,s0,None))
    

    while openH:
        
        g, s, p = heapq.heappop(openH) #skini prvi element s Heapa, onaj s najmanjom cijenom, Heap odrzava poredak
        n = (g,s,p) #cvor 'n' se sastoji od (ukupne cijene,stanja,roditelja)
        g = float(g)

        #ako stanje vec nije medu posjecenim stanjima.., moze se dogoditi da smo iz dva razlicita stanja dosli do istog stanja
        #samo se jedan prijelaz dogodio prije, uvijek zelimo onaj brzi prijelaz pa pamtimo samo prvog roditelja istog stanja
        if s not in visited:
            visited.add(s) 
            parents[s] = p
        
        for i in goal:
            if i == s:
                return reconstruct(i,parents),len(visited),g #vrati put,broj posjecenih stanja i ukupnu cijenu puta
   

        for item in graf[s].items():
            if item[0] not in visited:
                g_m, s_m, p_m = expand_cost(n,item) #pozovi funkciju za prosirenje cvora
                m = (g_m,s_m,p_m)
                heapq.heappush(openH,m) #stavi novo stanje na minheap, s time da zadrzava svoju strukturu
                


    return fail




#algoritam za prosirivanje cvorova za slucaj A* algoritma
def Astar_expand(n,next,h_next):
    

    h_n, c_n, s_n, p_n = n #heuristika,cijena,stanje,roditelj
    s_next, c_next = next
    
    
    h_next = float(h_next)
    c_n = float(c_n)
    
    real_cost = c_n + c_next #stvarna cijena
    f_n = real_cost + h_next #ukupna cijena f(n)
    


    return f_n,real_cost,s_next,s_n #f(n),stvarna cijena,iduce_stanje,roditelj



#algoritam za pretrazivanje A*
def aStarSearch(s0,goal,graf,heuristic_func):
    
    
    
    closed = set()
    openH = []
    parents = {}
    
    

    #dodaj tuple u Heap(heuristika+,prava_cijena,stanje,roditelj), prvi clan mora biti vrijednost heuristicke fje
    #jer zelim da uvijek prvo skidam onaj element sa najmanjom vrijednosti  ukupne cijene
    openH.append((heuristic_func[s0],0,s0,None))
    

    while openH:
        #skini heuristiku,pravu cijenu puta i stanje iduceg cvora sa Heapa
        h,g,s,p = heapq.heappop(openH)
        #spremi te podatke kao tuple
        n = (h,g,s,p)
        
        
        #ako nije u zatvorenim cvorovima
        if s not in closed:
            parents[s] = p
            closed.add(s)

        
        for i in goal:
            if i == s:
                g = float(g)
                return reconstruct(i,parents),len(closed),g #vrati put,broj zatvorenih cvorova, ukupnu cijenu

        for item in graf[s].items():
            if item[0] not in closed:
                h_m, g_m, s_m, p_m = Astar_expand(n,item,heuristic_func[item[0]]) 
                heapq.heappush(openH,(h_m,g_m,s_m,p_m)) #dodaj na minHeap tako da se odrzi poredak, cvor s najmanjom f(n) je prvi
             
    return fail

#provjeri je li heuristika optimisticna
def Is_heuristic_optimistic(goal,graf,heuristic_func):

    j=0
    i=0
    
    #provjeravamo za svako stanje u prostoru stanja
    while j < len(graf):
        for item in graf.items():
            h_state = heuristic_func[item[0]] #heuristika trenutnog stanja
            h_state = float(h_state)
            x,y,h_star = aStarSearch(item[0],goal,graf,heuristic_func) #trazimo optimalan put od pojedinog stanja do ciljnog stanja

            #ako je heuristika pojedinog stanja veca od vrijendosti 'perfektne' heuristike, prekrseno je stanje optimisticnosti
            if h_state > h_star:
                print("h({}) > h*: {} > {}".format(item[0],h_state,h_star))
                i += 1

            j += 1
            
            
    if i == 0:
        print("Heuristic is optimistic")
    else:
        print("Heuristic is not optimistic")
        print("{} errors found".format(i))
        
    
#provjeri je li heuristika konzistentna
def Is_heuristic_consistent(start,goal,graf,heuristic_func):
    e = 0
    i = False

    for parent in graf:
        for item in graf[parent].items():
            s_next, c_next = item
            h_s1 = heuristic_func[parent] #heuristika trenutnog stanja
            h_s2 = heuristic_func[s_next] #heuristika iduceg stanja
            h_s1 = float(h_s1)
            h_s2 = float(h_s2)
            c_next = float(c_next)

            #ako je heuristika trenutnog stanja veca od zbroja heuristike iduceg i cijene prijelaza izmedu onda se krsi konzistentnost
            if h_s1 > h_s2 + c_next:
                e += 1
                print("h({}) > h({}) + c: {} > {} + {}".format(parent,s_next,h_s1,h_s2,c_next))
                i = True
        
    if i:
        print("Heuristic is not consistent")
        print("Found {} errors".format(e))
    else:
        print("Heuristic is consistent")
            
            
    



if __name__ == "__main__":

    states = []
    counter = 0
    q=0
    

    #otvara datoteku za citanje
    with open("istra.txt","r") as f:
            
        
        read = f.readlines()
        
        #ako linija pocinje sa '#' ignorira ju
        while counter < len(read):
            line = read[counter]
            
            
            if not line.startswith('#'):
                file_start = counter
                break

            counter += 1

    #napravi novu listu sa  stanjima grafa bez # te pocetnog i krajnjih stanja
    for currentline in read[file_start+2:]:
        states.append(currentline)
                    
    #zapisi pocetno stanje
    start = read[file_start].strip()
    
    #dodajemo ciljna stanja u listu, cijepamo string u listu(moze biti vise konacnih stanja)
    goal = read[file_start+1].strip().split(" ")

    #pozovi funkciju da napravi graf
    graf = make_graph(states)
    
    #iteriraj po nested rijecniku da bi saznao broj cvorova djece koji je jednak broju mogucih prijelaza
    for parent in graf:
        for item in graf[parent].items():
            q += 1
    


    #spremi heuristiku
    heuristic_func = load_heuristics()


    print("Start state: {}".format(start))
    print("End state(s): {}".format(goal))
    print("State space size: {}".format(len(states)))
    print("Total transitions: {}".format(q))
    print("\n")



    
    #provjeri je li heuristika optimisticna
    print("Checking if heuristic is optimistic:")
    Is_heuristic_optimistic(goal,graf,heuristic_func)
    #provjeri je li heuristika konzistentna
    print("\n")
    print("Checking if heuristic is consistent:")
    Is_heuristic_consistent(start,goal,graf,heuristic_func)
    print("\n")
    
    print("Running Astar:")
    #pozovi funkciju za A* algoritam
    travel_path, states_visited, total_cost = aStarSearch(start,goal,graf,heuristic_func)
    path_lenght = len(travel_path)
    print("States visited: {}".format(states_visited))
    print("Found path of lenght: {}".format(path_lenght))
    total_cost = float(total_cost)
    print("Total cost: {}".format(total_cost))
    print("==>")
    for i in travel_path:
        print("{}".format(i))
       

    print("\n")
    print("Running BFS:")
    #pozivaj funkciju za BFS redom za sva ciljna stanja,f-ja vraca put od pocetnog stanja do ciljnog
    travel_path, states_visited = BreadthFirstSearch(start,goal,graf)
    path_lenght = len(travel_path)
    print("States visited: {}".format(states_visited))
    print("Found path of lenght: {}".format(path_lenght))
    print("==>")
    for i in travel_path:
        print("{}".format(i))

    print("\n")
    print("Running UCS:")
    travel_path, states_visited,total_cost = UniformCostSearch(start,goal,graf)
    path_lenght = len(travel_path)
    print("States visited: {}".format(states_visited))
    print("Found path of lenght: {}".format(path_lenght))
    total_cost = float(total_cost)
    print("Total cost: {}".format(total_cost))
    print("==>")
    for i in travel_path:
        print("{}".format(i))

pass
    
