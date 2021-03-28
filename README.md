# FloriBot4.0

Dieses Repository umfasst den Softwarestand des Robotoers FloriBot 4.0.
FloriBot 4.0 ist ein autonomer mobiler Roboter, welcher von Studenten der Hoschule Heilbronn entwicklet und aufgebaut wird. 
In diesem Repository wird die Software des Roboter versioniert und verwaltet. 
Der Roboter wurde entwicklet, um die Aufgabe des [Field-Robot-Events](https://www.fieldrobot.com/ "Hompage des Events") zu meistern. 
Die Software basiert auf der Middleware [ROS](https://www.ros.org/ "ROS Homepage") Melodic und wird auf einem Ubuntu 18.04 System ausgeführt.
Um die Entwicklung einheitlich und strukturiert zu halten, werden im Folgenden die Regel innerhalb des Repositorys erläutert. 

## Regeln

Nachfolgend werden die Regeln innerhalb des Repositorys dargelegt.

* Als zulässige Entwicklungssprache stehen Python und C++ , sowie Matlab und Simulik zur verfügung.
* Für die eigene Softwareentwicklung muss ein Feature-Branch erstell werden.
* Der Master-Branche beinhaltet den aktuellen Softwarestand des Roboter. 
* Der Develop-Branch dient zum Testen der eigenen Software in Kombination mit allen anderen Komponenten.
* Der Master-, sowie der Develop-Branch können nur mittel eines Pullrequests verändert werden. 
* Änderung des Master-Branch ist nur über des Develop-Branch möglich.
* Bevor der eigene Feature-Branch mit dem Develop-Branch vereint werden darf, muss nachgewiesen werden, dass es keine Fehler im Code gibt und der, im Wiki des Repos eingetragene Styleguid eingehalten wurde.
* Um eine Softwareänderung zu commiten, ist das beigefügte Template zu verwenden. 
* Eine Nachricht (Commit-Message) sollte immer eine kurze Umschreibung des Commits, eine Beschreibung des Problems, die eingefügte Lösung sowie eventuelle Anmerkungen enthalten. 
* Innerhalb des .gitignore-Files darf nichts verändert werden.
* Da es sich hierbei um eine Softwarentwicklung auf ROS-Basis handelt, sind nur die Package-Ordner (alles in catkin/src) des Workspace hochzuladen. 
* IDE-Spezifische Ordnerstuckturen, wie beispielsweise bei VS-Code der .vscode-Odrner sind nicht zulässig

## Setup Repostory

Der folgende Abschnitt beschreibt das erste Verwenden des Repositorys, sowie das Einfügen der nötigen Einstellungen. 
Detailierte Angaben, sowie der Codeing-Styleguid befinden sich im Wiki.

### Clonen des Repositorys

In diesem Abschnitt wird nur das Auschecken des Repositorys mittel Git erklärt. Auf Möglichkeiten, wie dies in Matlab umzusetzten ist wird nicht eingegangen. 

Nach dem Installieren von [Git](https://git-scm.com/downloads), muss in den src-Pfad des Workspace navigiert werden, in welchem das Projekt später mit catkin_make gebaut werden soll. 
Mit Rechtsklik kann ein Git-Terminal geöffnet werden. Mit dem Befehl ``git clone https://github.com/Team-FloriBot/FloriBot4.0.git`` wird das Repository vom Master-Branch lokal gespeichert wird. 

### Commit Template

Um den Softwarestand dauerhaft, gut dokumentiert zu halten, wurde ein Template erstellt, welches das Commiten vereinfacht. 
Hierfür muss ein Terminal im Git-Workspace geöffnet werden. 
Neben dem .gitignore-File existiert auch ein .gitmessage-File. Dieses beinhaltet das Template für die Commitmessages. 
Mit dem Befehl ``git config --local commit.template .gitmessage`` wird es in Git eingebunden. 
Mittels ``git commit`` kann das Committen ausgelöst werden. Es wird ein Fenster geöffnet, welches eine Vorlage enthält. 
Hier müssen die Punkte dem Schema entsprechend angepasst und gespeichert werden. Zeilen, welche mit einen ``#`` anfangen werden in der späteren Message ignoriert.
Durch das Speichern der Datei wird der Commit verfolständigt.
