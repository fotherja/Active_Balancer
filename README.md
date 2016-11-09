# Active-LiPo-Balancer
Here is the software for an <b>active LiPo balancer</b> that <b>can handle 7 cells</b> and is <b>daisy chainable</b>. The term "active" means that the circuit uses DC/DC converters to suck charge from cells with a higher voltage and push it into cells with a lower voltage. This process brings all cells to an average cell voltage. This is 
<ul>
  <li>More efficient than passive balancers that simply dump excess charge into a resistive load</li>
  <li>Able to maintain balance during use if left connected so that the full capacity of the battery can be utilised</li>
</ul>
With typical balancers, even if a battery starts balanced the BMS must stop further power draw from the battery whenever the weakest cell reaches the minimum cutoff voltage. It may well be the case that other cells still have plenty of energy to give so the battery as a whole is at the mercy of the weakest cell and the full potential isn't reached. An in situ balancer can attempt to maintain balance and hence the most can be got out of the battery.   

<br>

<p align="center">
  <img src="http://www.jamesfotherby.com/Images/Balancer/Circuit_No_Border.jpg" width="60%">
</p>

<br>

This is the circuit. The 3 visable chips are dual H-Bridge drivers which allows for 6 DC/DC converters, however 7 cells can be balanced. For the full design including Schematic, Component BOM and PCB layouts, see the <a href = "https://upverter.com/fotherja/802f175a0b0332e8/6S-Dynamic-LiPo-Balancer/">Upverter Page</a> 

This project was only partially successful. The ADCs didn't have sufficient resolution for this circuit to be good enough and it was only balanceing to within about 0.1volts. For more details see <a href="http://www.jamesfotherby.com/Balancer.html">my website page</a>

