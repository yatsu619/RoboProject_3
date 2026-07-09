<!-- Geschrieben und dokumentiert von Johannes Sedlmayr -->

# Implementierungsentscheidungen

Dieses Dokument beschreibt die wesentlichen Entwurfsentscheidungen der drei Softwarekomponenten.
---

# Predic_logic

## Berechnung der Förderbandgeschwindigkeit

Die Förderbandgeschwindigkeit wird aus der Positionsänderung zwischen zwei aufeinanderfolgenden Messungen bestimmt. Dabei wird angenommen, dass sich das Förderband während des betrachteten Zeitraums mit konstanter Geschwindigkeit bewegt. Da das Förderband im Normalbetrieb nahezu gleichförmig läuft, können Beschleunigungs- und Verzögerungsvorgänge vernachlässigt werden.
Da Positionsmessungen Messrauschen und einzelne Ausreißer enthalten können, wird nicht die Momentangeschwindigkeit direkt verwendet. Stattdessen werden mehrere Geschwindigkeitswerte in einem Ringpuffer gespeichert und anschließend mithilfe eines Medianfilters ausgewertet.

**Der Median wurde gewählt, weil er:**

- robust gegenüber Ausreißern ist,
- Fehlmessungen zuverlässig unterdrückt,
- eine stabile Geschwindigkeitsbestimmung ermöglicht.

Positive Geschwindigkeiten werden verworfen, da sich das Förderband ausschließlich in negativer Richtung bewegt. Die Messwerte werden in einer `deque` mit fester Fenstergröße gespeichert, wodurch ältere Werte automatisch entfernt werden und der Speicherbedarf konstant bleibt.

---

# Waypoint_prediction

## Verwendung eines Timers

Die Kamera liefert ihre Daten asynchron und nur dann, wenn Objekte erkannt werden. Dadurch entstehen unregelmäßige Aktualisierungsintervalle.
Die Verarbeitung erfolgt deshalb in einem periodischen Timer und nicht direkt im Callback. Dadurch arbeitet der Knoten unabhängig von der Veröffentlichungsrate der Kamera und besitzt eine konstante Zykluszeit.

**Vorteile**

- gleichmäßige Verarbeitung
- unabhängig von der Kamerafrequenz
- keine Weitergabe von Verzögerungen

---

## Veröffentlichung der Objektdaten

Die berechneten Objektinformationen werden erst veröffentlicht, wenn das Objekt den Erfassungsbereich der Kamera verlässt. Zu diesem Zeitpunkt liegen alle relevanten Informationen vollständig vor und der Roboter benötigt sie erst für den späteren Greifvorgang.

**Vorteile**

- vollständiger Datensatz
- geringerer Nachrichtenverkehr
- einfachere Synchronisation

---

## Medianfilter

Sowohl für die Förderbandgeschwindigkeit als auch für die y-Position wird ein Medianfilter verwendet. Während die y-Position idealerweise konstant bleibt und lediglich durch Kamerarauschen beeinflusst wird, reagiert die Geschwindigkeitsberechnung empfindlich auf kleine Positionsfehler. Der Median reduziert diese Ausreißer und liefert stabile Werte.

---

## Bestimmung des Objekttyps

Die Kamera liefert erkannte Objekttypen nicht in einer festen Reihenfolge. Während mehrere Objekte gleichzeitig sichtbar sind, können dieselben Objekttypen mehrfach zwischen anderen Erkennungen auftreten.
Deshalb wird der am häufigsten erkannte Objekttyp verwendet, da dieses Objekt typischerweise am längsten im Sichtfeld der Kamera verbleibt und somit als nächstes verarbeitet werden muss.
Zur Behandlung unmittelbar aufeinanderfolgender Objekte desselben Typs wird zusätzlich `last_obj_type` gespeichert. Dadurch bleibt die Objektzuordnung auch dann erhalten, wenn beim Entfernen eines Objekttyps mehrere identische Einträge aus dem Puffer gelöscht werden.

---

# Prediction_delay

## FIFO-Puffer

Nach der Vorverarbeitung werden alle Objekte in einem FIFO-Puffer (`deque`) gespeichert. Neue Objekte werden hinten eingefügt und bereits bearbeitete Objekte vorne entnommen. Dadurch entspricht die Reihenfolge der Verarbeitung der tatsächlichen Reihenfolge auf dem Förderband.
Die Verwendung einer `deque` ermöglicht zudem Einfügen und Entnehmen mit konstanter Laufzeit und eignet sich daher für kontinuierliche Echtzeitanwendungen.

---

## Aktives Objekt

Obwohl sich mehrere Objekte gleichzeitig auf dem Förderband befinden können, wird immer nur ein Objekt aktiv verarbeitet. Nach einem erfolgreichen Greifvorgang wird automatisch das nächste Objekt aus dem Puffer übernommen. Dieses Vorgehen entspricht dem tatsächlichen Arbeitsablauf des Roboters und vereinfacht die Zustandsverwaltung.

---

## Zeitbasierte Positionsvorhersage

Nach dem Verlassen des Kamerabereichs stehen keine weiteren Messwerte zur Verfügung. Die aktuelle Objektposition wird deshalb anhand der letzten bekannten Position, der Förderbandgeschwindigkeit und der vergangenen Zeit bestimmt.

```text
x(t) = x₀ + vₓ · Δt
```

Dadurch kann die Objektposition bis zum Greifpunkt zuverlässig vorhergesagt werden.

---

## Timer und Veröffentlichung

Die Positionsvorhersage wird periodisch berechnet und erst veröffentlicht, sobald sich das Objekt innerhalb des definierten Greifbereichs befindet. Dadurch erhält die Robotersteuerung ausschließlich relevante Positionsinformationen und unnötige Nachrichten werden vermieden.

---

## Synchronisation mit dem Greifer

Nach der Veröffentlichung bleibt das aktuelle Objekt aktiv, bis der Greifer den erfolgreichen Greifvorgang bestätigt. Erst danach wird das nächste Objekt verarbeitet.

Da das Greifersignal mehrere Timerzyklen anliegen kann, verhindert die Sperrvariable `gripper_lock`, dass derselbe Greifvorgang mehrfach ausgewertet wird.