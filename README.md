# Notatka
## Odpalanie nodea od Lidara
```bash
ros2 launch urg_node2 urg_node2.launch.py
```
## Buildowanie ws
Po kazdej zmianie w jakimkolwiek pliku wewnatrz src nalezy ponownie zbuildowac ws i zaktulizowac srodowisko.
Aby nie trzeba bylo w kolko wpisywac tych samych kilku komend stworzylem prosty skrypt i alias do niego, przez co w konsoli wystarczy teraz wpisac
```bash
zbuduj_ws
```
Jakby cos to skrypt znajduje sie na laptopie R4H w lokalizacji
```bash
~/skrypty
```
## Tworzenie drzewka tf
Aby drzewko nie tworzylo sie w biezacej lokalizacji tak jak jest to domyslnie stworzylem prosty skrypt, ktory tworzy wszystkie drzewka
w lokalizacji
```bash
~/ros2_ws/drzewka_tf
```
kazde drzewko zapisuje sie w katalogu ktory ma w swojej nazwie date wygenerowania drzewka
