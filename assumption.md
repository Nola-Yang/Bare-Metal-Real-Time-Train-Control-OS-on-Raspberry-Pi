# assumptions for TC1
- at the start of the control system: we assume the train stop at the loop, exact position unknown, direction unknown
- will use level 8 speed for goto command (for stopping distance and actual velosity measuremnts)
- the bigger loop will be used to gain stable speed (for track A: `A3 → C13 → E7 → D7 → D9 → E12 → D11 → C16 → C6 → B15 → A3 ...`)
- no 2 consecutive sensors are broken
- some switch may work after the sent repeated commands
- loop switch will be set at init process
- from any sensor node, the loop is reachable in at least one direction (forward or reverse); KASSERT fires if neither works
- goto：  
  - **First**: train stop at the loop, exact position unknown, direction unknown → sensor trigger → get direction → stable spped → excute the route → stopped  
  - **second**: with exact train position and direction 
    -  if outside of the loop, back to loop to get stable speed
    -  restart the train
  - no reverse command will be used during train running process. may be used at stop
  - the switches in the loop will be re-set over and over again once get signal from sensor in the loop
  
---
