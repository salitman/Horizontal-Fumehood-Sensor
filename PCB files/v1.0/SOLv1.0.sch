<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="ESP32-C3-WROOM-02-N4">
<packages>
<package name="MODULE_ESP32-C3-WROOM-02-H4">
<wire x1="-9" y1="10" x2="-9" y2="4" width="0.127" layer="51"/>
<wire x1="-9" y1="4" x2="-9" y2="-10" width="0.127" layer="51"/>
<wire x1="-9" y1="-10" x2="9" y2="-10" width="0.127" layer="51"/>
<wire x1="9" y1="-10" x2="9" y2="4" width="0.127" layer="51"/>
<wire x1="9" y1="4" x2="9" y2="10" width="0.127" layer="51"/>
<wire x1="9" y1="10" x2="-9" y2="10" width="0.127" layer="51"/>
<wire x1="9" y1="4" x2="-9" y2="4" width="0.127" layer="51"/>
<wire x1="-9" y1="-10" x2="-9" y2="-9.87" width="0.127" layer="21"/>
<wire x1="9" y1="-10" x2="9" y2="-9.87" width="0.127" layer="21"/>
<wire x1="-9" y1="10" x2="-9" y2="3.67" width="0.127" layer="21"/>
<wire x1="-9" y1="-10" x2="9" y2="-10" width="0.127" layer="21"/>
<wire x1="9" y1="3.67" x2="9" y2="10" width="0.127" layer="21"/>
<wire x1="9" y1="10" x2="-9" y2="10" width="0.127" layer="21"/>
<wire x1="-9.75" y1="10.25" x2="-9.75" y2="-10.25" width="0.05" layer="39"/>
<wire x1="-9.75" y1="-10.25" x2="9.75" y2="-10.25" width="0.05" layer="39"/>
<wire x1="9.75" y1="-10.25" x2="9.75" y2="10.25" width="0.05" layer="39"/>
<wire x1="9.75" y1="10.25" x2="-9.75" y2="10.25" width="0.05" layer="39"/>
<circle x="-10.5" y="2.9" radius="0.1" width="0.2" layer="21"/>
<circle x="-10.5" y="2.9" radius="0.1" width="0.2" layer="51"/>
<text x="-9.75" y="10.45" size="1.27" layer="25">&gt;NAME</text>
<text x="-9.75" y="-10.45" size="1.27" layer="27" align="top-left">&gt;VALUE</text>
<rectangle x1="-9" y1="4" x2="9" y2="10" layer="41"/>
<rectangle x1="-9" y1="4" x2="9" y2="10" layer="43"/>
<rectangle x1="-9" y1="4" x2="9" y2="10" layer="42"/>
<smd name="1" x="-8.75" y="2.9" dx="1.5" dy="0.9" layer="1"/>
<smd name="2" x="-8.75" y="1.4" dx="1.5" dy="0.9" layer="1"/>
<smd name="3" x="-8.75" y="-0.1" dx="1.5" dy="0.9" layer="1"/>
<smd name="4" x="-8.75" y="-1.6" dx="1.5" dy="0.9" layer="1"/>
<smd name="5" x="-8.75" y="-3.1" dx="1.5" dy="0.9" layer="1"/>
<smd name="6" x="-8.75" y="-4.6" dx="1.5" dy="0.9" layer="1"/>
<smd name="7" x="-8.75" y="-6.1" dx="1.5" dy="0.9" layer="1"/>
<smd name="8" x="-8.75" y="-7.6" dx="1.5" dy="0.9" layer="1"/>
<smd name="9" x="-8.75" y="-9.1" dx="1.5" dy="0.9" layer="1"/>
<smd name="10" x="8.75" y="-9.1" dx="1.5" dy="0.9" layer="1"/>
<smd name="11" x="8.75" y="-7.6" dx="1.5" dy="0.9" layer="1"/>
<smd name="12" x="8.75" y="-6.1" dx="1.5" dy="0.9" layer="1"/>
<smd name="13" x="8.75" y="-4.6" dx="1.5" dy="0.9" layer="1"/>
<smd name="14" x="8.75" y="-3.1" dx="1.5" dy="0.9" layer="1"/>
<smd name="15" x="8.75" y="-1.6" dx="1.5" dy="0.9" layer="1"/>
<smd name="16" x="8.75" y="-0.1" dx="1.5" dy="0.9" layer="1"/>
<smd name="17" x="8.75" y="1.4" dx="1.5" dy="0.9" layer="1"/>
<smd name="18" x="8.75" y="2.9" dx="1.5" dy="0.9" layer="1"/>
<smd name="23" x="0.96" y="-3.3" dx="0.7" dy="0.7" layer="1"/>
<smd name="24" x="2.06" y="-3.3" dx="0.7" dy="0.7" layer="1"/>
<smd name="22" x="-0.14" y="-3.3" dx="0.7" dy="0.7" layer="1"/>
<smd name="19" x="-0.14" y="-2.2" dx="0.7" dy="0.7" layer="1"/>
<smd name="20" x="0.96" y="-2.2" dx="0.7" dy="0.7" layer="1"/>
<smd name="21" x="2.06" y="-2.2" dx="0.7" dy="0.7" layer="1"/>
<smd name="27" x="2.06" y="-4.4" dx="0.7" dy="0.7" layer="1"/>
<smd name="26" x="0.96" y="-4.4" dx="0.7" dy="0.7" layer="1"/>
<smd name="25" x="-0.14" y="-4.4" dx="0.7" dy="0.7" layer="1"/>
<pad name="29" x="1.51" y="-2.2" drill="0.3" diameter="0.4" stop="no"/>
<pad name="32" x="2.06" y="-2.75" drill="0.3" diameter="0.4" stop="no"/>
<pad name="37" x="2.06" y="-3.85" drill="0.3" diameter="0.4" stop="no"/>
<pad name="28" x="0.41" y="-2.2" drill="0.3" diameter="0.4" stop="no"/>
<pad name="34" x="1.51" y="-3.3" drill="0.3" diameter="0.4" stop="no"/>
<pad name="31" x="0.96" y="-2.75" drill="0.3" diameter="0.4" stop="no"/>
<pad name="30" x="-0.14" y="-2.75" drill="0.3" diameter="0.4" stop="no"/>
<pad name="33" x="0.41" y="-3.3" drill="0.3" diameter="0.4" stop="no"/>
<pad name="39" x="1.51" y="-4.4" drill="0.3" diameter="0.4" stop="no"/>
<pad name="38" x="0.41" y="-4.4" drill="0.3" diameter="0.4" stop="no"/>
<pad name="36" x="0.96" y="-3.85" drill="0.3" diameter="0.4" stop="no"/>
<pad name="35" x="-0.14" y="-3.85" drill="0.3" diameter="0.4" stop="no"/>
</package>
</packages>
<symbols>
<symbol name="ESP32-C3-WROOM-02-N4">
<wire x1="-12.7" y1="20.32" x2="12.7" y2="20.32" width="0.254" layer="94"/>
<wire x1="12.7" y1="-17.78" x2="12.7" y2="20.32" width="0.254" layer="94"/>
<wire x1="12.7" y1="-17.78" x2="-12.7" y2="-17.78" width="0.254" layer="94"/>
<wire x1="-12.7" y1="20.32" x2="-12.7" y2="-17.78" width="0.254" layer="94"/>
<text x="-12.7" y="21.082" size="1.778" layer="95">&gt;NAME</text>
<text x="-12.7" y="-18.542" size="1.778" layer="96" align="top-left">&gt;VALUE</text>
<pin name="3V3" x="17.78" y="17.78" length="middle" direction="pwr" rot="R180"/>
<pin name="EN" x="-17.78" y="15.24" length="middle" direction="in"/>
<pin name="GND" x="17.78" y="-15.24" length="middle" direction="pwr" rot="R180"/>
<pin name="IO0" x="-17.78" y="10.16" length="middle"/>
<pin name="IO1" x="-17.78" y="7.62" length="middle"/>
<pin name="IO2" x="-17.78" y="5.08" length="middle"/>
<pin name="IO3" x="-17.78" y="2.54" length="middle"/>
<pin name="IO4" x="-17.78" y="0" length="middle"/>
<pin name="IO5" x="-17.78" y="-2.54" length="middle"/>
<pin name="IO6" x="-17.78" y="-5.08" length="middle"/>
<pin name="IO7" x="17.78" y="10.16" length="middle" rot="R180"/>
<pin name="IO8" x="17.78" y="7.62" length="middle" rot="R180"/>
<pin name="IO9" x="17.78" y="5.08" length="middle" rot="R180"/>
<pin name="IO10" x="17.78" y="2.54" length="middle" rot="R180"/>
<pin name="IO18" x="17.78" y="0" length="middle" rot="R180"/>
<pin name="IO19" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="TXD" x="-17.78" y="-10.16" length="middle"/>
<pin name="RXD" x="-17.78" y="-12.7" length="middle"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="ESP32-C3-WROOM-02-N4" prefix="U">
<description>WiFi Modules (802.11) (Engineering Samples) SMD module, ESP32-C3, 4MB SPI flash, PCB antenna, -40 C +105 C   &lt;a href="https://pricing.snapeda.com/parts/ESP32-C3-WROOM-02-H4/Espressif%20Systems/view-part?ref=eda"&gt;Check availability&lt;/a&gt;  &lt;a href="https://pricing.snapeda.com/parts/ESP32-C3-WROOM-02-N4/Espressif%20Systems/view-part?ref=eda"&gt;Check availability&lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="ESP32-C3-WROOM-02-N4" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MODULE_ESP32-C3-WROOM-02-H4">
<connects>
<connect gate="G$1" pin="3V3" pad="1"/>
<connect gate="G$1" pin="EN" pad="2"/>
<connect gate="G$1" pin="GND" pad="9 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39"/>
<connect gate="G$1" pin="IO0" pad="18"/>
<connect gate="G$1" pin="IO1" pad="17"/>
<connect gate="G$1" pin="IO10" pad="10"/>
<connect gate="G$1" pin="IO18" pad="13"/>
<connect gate="G$1" pin="IO19" pad="14"/>
<connect gate="G$1" pin="IO2" pad="16"/>
<connect gate="G$1" pin="IO3" pad="15"/>
<connect gate="G$1" pin="IO4" pad="3"/>
<connect gate="G$1" pin="IO5" pad="4"/>
<connect gate="G$1" pin="IO6" pad="5"/>
<connect gate="G$1" pin="IO7" pad="6"/>
<connect gate="G$1" pin="IO8" pad="7"/>
<connect gate="G$1" pin="IO9" pad="8"/>
<connect gate="G$1" pin="RXD" pad="11"/>
<connect gate="G$1" pin="TXD" pad="12"/>
</connects>
<technologies>
<technology name="">
<attribute name="AVAILABILITY" value="In Stock"/>
<attribute name="CHECK_PRICES" value="https://www.snapeda.com/parts/ESP32-C3-WROOM-02-N4/Espressif+Systems/view-part/?ref=eda"/>
<attribute name="DESCRIPTION" value=" WiFi Modules (802.11) (Engineering Samples) SMD module, ESP32-C3, 4MB SPI flash, PCB antenna, -40 C +85 C "/>
<attribute name="MF" value="Espressif Systems"/>
<attribute name="MP" value="ESP32-C3-WROOM-02-N4"/>
<attribute name="PACKAGE" value="Package "/>
<attribute name="PRICE" value="None"/>
<attribute name="SNAPEDA_LINK" value="https://www.snapeda.com/parts/ESP32-C3-WROOM-02-N4/Espressif+Systems/view-part/?ref=snap"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="rcl" urn="urn:adsk.eagle:library:334">
<description>&lt;b&gt;Resistors, Capacitors, Inductors&lt;/b&gt;&lt;p&gt;
Based on the previous libraries:
&lt;ul&gt;
&lt;li&gt;r.lbr
&lt;li&gt;cap.lbr 
&lt;li&gt;cap-fe.lbr
&lt;li&gt;captant.lbr
&lt;li&gt;polcap.lbr
&lt;li&gt;ipc-smd.lbr
&lt;/ul&gt;
All SMD packages are defined according to the IPC specifications and  CECC&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;&lt;p&gt;
&lt;p&gt;
for Electrolyt Capacitors see also :&lt;p&gt;
www.bccomponents.com &lt;p&gt;
www.panasonic.com&lt;p&gt;
www.kemet.com&lt;p&gt;
http://www.secc.co.jp/pdf/os_e/2004/e_os_all.pdf &lt;b&gt;(SANYO)&lt;/b&gt;
&lt;p&gt;
for trimmer refence see : &lt;u&gt;www.electrospec-inc.com/cross_references/trimpotcrossref.asp&lt;/u&gt;&lt;p&gt;

&lt;table border=0 cellspacing=0 cellpadding=0 width="100%" cellpaddding=0&gt;
&lt;tr valign="top"&gt;

&lt;! &lt;td width="10"&gt;&amp;nbsp;&lt;/td&gt;
&lt;td width="90%"&gt;

&lt;b&gt;&lt;font color="#0000FF" size="4"&gt;TRIM-POT CROSS REFERENCE&lt;/font&gt;&lt;/b&gt;
&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=2&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;RECTANGULAR MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BOURNS&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;BI&amp;nbsp;TECH&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;DALE-VISHAY&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PHILIPS/MEPCO&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MURATA&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;PANASONIC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;SPECTROL&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;B&gt;
      &lt;FONT SIZE=3 FACE=ARIAL color="#FF0000"&gt;MILSPEC&lt;/FONT&gt;
      &lt;/B&gt;
    &lt;/TD&gt;&lt;TD&gt;&amp;nbsp;&lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3 &gt;
      3005P&lt;BR&gt;
      3006P&lt;BR&gt;
      3006W&lt;BR&gt;
      3006Y&lt;BR&gt;
      3009P&lt;BR&gt;
      3009W&lt;BR&gt;
      3009Y&lt;BR&gt;
      3057J&lt;BR&gt;
      3057L&lt;BR&gt;
      3057P&lt;BR&gt;
      3057Y&lt;BR&gt;
      3059J&lt;BR&gt;
      3059L&lt;BR&gt;
      3059P&lt;BR&gt;
      3059Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      89P&lt;BR&gt;
      89W&lt;BR&gt;
      89X&lt;BR&gt;
      89PH&lt;BR&gt;
      76P&lt;BR&gt;
      89XH&lt;BR&gt;
      78SLT&lt;BR&gt;
      78L&amp;nbsp;ALT&lt;BR&gt;
      56P&amp;nbsp;ALT&lt;BR&gt;
      78P&amp;nbsp;ALT&lt;BR&gt;
      T8S&lt;BR&gt;
      78L&lt;BR&gt;
      56P&lt;BR&gt;
      78P&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      T18/784&lt;BR&gt;
      783&lt;BR&gt;
      781&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2199&lt;BR&gt;
      1697/1897&lt;BR&gt;
      1680/1880&lt;BR&gt;
      2187&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      8035EKP/CT20/RJ-20P&lt;BR&gt;
      -&lt;BR&gt;
      RJ-20X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      1211L&lt;BR&gt;
      8012EKQ&amp;nbsp;ALT&lt;BR&gt;
      8012EKR&amp;nbsp;ALT&lt;BR&gt;
      1211P&lt;BR&gt;
      8012EKJ&lt;BR&gt;
      8012EKL&lt;BR&gt;
      8012EKQ&lt;BR&gt;
      8012EKR&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      2101P&lt;BR&gt;
      2101W&lt;BR&gt;
      2101Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      2102L&lt;BR&gt;
      2102S&lt;BR&gt;
      2102Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVMCOG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      43P&lt;BR&gt;
      43W&lt;BR&gt;
      43Y&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      40L&lt;BR&gt;
      40P&lt;BR&gt;
      40Y&lt;BR&gt;
      70Y-T602&lt;BR&gt;
      70L&lt;BR&gt;
      70P&lt;BR&gt;
      70Y&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      RT/RTR12&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;
      RJ/RJR12&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SQUARE MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
   &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3250L&lt;BR&gt;
      3250P&lt;BR&gt;
      3250W&lt;BR&gt;
      3250X&lt;BR&gt;
      3252P&lt;BR&gt;
      3252W&lt;BR&gt;
      3252X&lt;BR&gt;
      3260P&lt;BR&gt;
      3260W&lt;BR&gt;
      3260X&lt;BR&gt;
      3262P&lt;BR&gt;
      3262W&lt;BR&gt;
      3262X&lt;BR&gt;
      3266P&lt;BR&gt;
      3266W&lt;BR&gt;
      3266X&lt;BR&gt;
      3290H&lt;BR&gt;
      3290P&lt;BR&gt;
      3290W&lt;BR&gt;
      3292P&lt;BR&gt;
      3292W&lt;BR&gt;
      3292X&lt;BR&gt;
      3296P&lt;BR&gt;
      3296W&lt;BR&gt;
      3296X&lt;BR&gt;
      3296Y&lt;BR&gt;
      3296Z&lt;BR&gt;
      3299P&lt;BR&gt;
      3299W&lt;BR&gt;
      3299X&lt;BR&gt;
      3299Y&lt;BR&gt;
      3299Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      -&lt;BR&gt;
      64P&amp;nbsp;ALT&lt;BR&gt;
      64W&amp;nbsp;ALT&lt;BR&gt;
      64X&amp;nbsp;ALT&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      66X&amp;nbsp;ALT&lt;BR&gt;
      66P&amp;nbsp;ALT&lt;BR&gt;
      66W&amp;nbsp;ALT&lt;BR&gt;
      66P&lt;BR&gt;
      66W&lt;BR&gt;
      66X&lt;BR&gt;
      67P&lt;BR&gt;
      67W&lt;BR&gt;
      67X&lt;BR&gt;
      67Y&lt;BR&gt;
      67Z&lt;BR&gt;
      68P&lt;BR&gt;
      68W&lt;BR&gt;
      68X&lt;BR&gt;
      67Y&amp;nbsp;ALT&lt;BR&gt;
      67Z&amp;nbsp;ALT&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      5050&lt;BR&gt;
      5091&lt;BR&gt;
      5080&lt;BR&gt;
      5087&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T63YB&lt;BR&gt;
      T63XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      5887&lt;BR&gt;
      5891&lt;BR&gt;
      5880&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T93Z&lt;BR&gt;
      T93YA&lt;BR&gt;
      T93XA&lt;BR&gt;
      T93YB&lt;BR&gt;
      T93XB&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKW&lt;BR&gt;
      8026EKM&lt;BR&gt;
      8026EKP&lt;BR&gt;
      8026EKB&lt;BR&gt;
      8026EKM&lt;BR&gt;
      1309X&lt;BR&gt;
      1309P&lt;BR&gt;
      1309W&lt;BR&gt;
      8024EKP&lt;BR&gt;
      8024EKW&lt;BR&gt;
      8024EKN&lt;BR&gt;
      RJ-9P/CT9P&lt;BR&gt;
      RJ-9W&lt;BR&gt;
      RJ-9X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      3103P&lt;BR&gt;
      3103Y&lt;BR&gt;
      3103Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3105P/3106P&lt;BR&gt;
      3105W/3106W&lt;BR&gt;
      3105X/3106X&lt;BR&gt;
      3105Y/3106Y&lt;BR&gt;
      3105Z/3105Z&lt;BR&gt;
      3102P&lt;BR&gt;
      3102W&lt;BR&gt;
      3102X&lt;BR&gt;
      3102Y&lt;BR&gt;
      3102Z&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMCBG&lt;BR&gt;
      EVMCCG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      55-1-X&lt;BR&gt;
      55-4-X&lt;BR&gt;
      55-3-X&lt;BR&gt;
      55-2-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      50-2-X&lt;BR&gt;
      50-4-X&lt;BR&gt;
      50-3-X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      64P&lt;BR&gt;
      64W&lt;BR&gt;
      64X&lt;BR&gt;
      64Y&lt;BR&gt;
      64Z&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RT/RTR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RJ/RJR22&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RT/RTR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RJ/RJR26&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RT/RTR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      RJ/RJR24&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=8&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MURATA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;SPECTROL&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD ALIGN=CENTER&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;MILSPEC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3323P&lt;BR&gt;
      3323S&lt;BR&gt;
      3323W&lt;BR&gt;
      3329H&lt;BR&gt;
      3329P&lt;BR&gt;
      3329W&lt;BR&gt;
      3339H&lt;BR&gt;
      3339P&lt;BR&gt;
      3339W&lt;BR&gt;
      3352E&lt;BR&gt;
      3352H&lt;BR&gt;
      3352K&lt;BR&gt;
      3352P&lt;BR&gt;
      3352T&lt;BR&gt;
      3352V&lt;BR&gt;
      3352W&lt;BR&gt;
      3362H&lt;BR&gt;
      3362M&lt;BR&gt;
      3362P&lt;BR&gt;
      3362R&lt;BR&gt;
      3362S&lt;BR&gt;
      3362U&lt;BR&gt;
      3362W&lt;BR&gt;
      3362X&lt;BR&gt;
      3386B&lt;BR&gt;
      3386C&lt;BR&gt;
      3386F&lt;BR&gt;
      3386H&lt;BR&gt;
      3386K&lt;BR&gt;
      3386M&lt;BR&gt;
      3386P&lt;BR&gt;
      3386S&lt;BR&gt;
      3386W&lt;BR&gt;
      3386X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      25P&lt;BR&gt;
      25S&lt;BR&gt;
      25RX&lt;BR&gt;
      82P&lt;BR&gt;
      82M&lt;BR&gt;
      82PA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      91E&lt;BR&gt;
      91X&lt;BR&gt;
      91T&lt;BR&gt;
      91B&lt;BR&gt;
      91A&lt;BR&gt;
      91V&lt;BR&gt;
      91W&lt;BR&gt;
      25W&lt;BR&gt;
      25V&lt;BR&gt;
      25P&lt;BR&gt;
      -&lt;BR&gt;
      25S&lt;BR&gt;
      25U&lt;BR&gt;
      25RX&lt;BR&gt;
      25X&lt;BR&gt;
      72XW&lt;BR&gt;
      72XL&lt;BR&gt;
      72PM&lt;BR&gt;
      72RX&lt;BR&gt;
      -&lt;BR&gt;
      72PX&lt;BR&gt;
      72P&lt;BR&gt;
      72RXW&lt;BR&gt;
      72RXL&lt;BR&gt;
      72X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      T7YB&lt;BR&gt;
      T7YA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      TXD&lt;BR&gt;
      TYA&lt;BR&gt;
      TYP&lt;BR&gt;
      -&lt;BR&gt;
      TYD&lt;BR&gt;
      TX&lt;BR&gt;
      -&lt;BR&gt;
      150SX&lt;BR&gt;
      100SX&lt;BR&gt;
      102T&lt;BR&gt;
      101S&lt;BR&gt;
      190T&lt;BR&gt;
      150TX&lt;BR&gt;
      101&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      101SX&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ET6P&lt;BR&gt;
      ET6S&lt;BR&gt;
      ET6X&lt;BR&gt;
      RJ-6W/8014EMW&lt;BR&gt;
      RJ-6P/8014EMP&lt;BR&gt;
      RJ-6X/8014EMX&lt;BR&gt;
      TM7W&lt;BR&gt;
      TM7P&lt;BR&gt;
      TM7X&lt;BR&gt;
      -&lt;BR&gt;
      8017SMS&lt;BR&gt;
      -&lt;BR&gt;
      8017SMB&lt;BR&gt;
      8017SMA&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      CT-6W&lt;BR&gt;
      CT-6H&lt;BR&gt;
      CT-6P&lt;BR&gt;
      CT-6R&lt;BR&gt;
      -&lt;BR&gt;
      CT-6V&lt;BR&gt;
      CT-6X&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKV&lt;BR&gt;
      -&lt;BR&gt;
      8038EKX&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      8038EKP&lt;BR&gt;
      8038EKZ&lt;BR&gt;
      8038EKW&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3321H&lt;BR&gt;
      3321P&lt;BR&gt;
      3321N&lt;BR&gt;
      1102H&lt;BR&gt;
      1102P&lt;BR&gt;
      1102T&lt;BR&gt;
      RVA0911V304A&lt;BR&gt;
      -&lt;BR&gt;
      RVA0911H413A&lt;BR&gt;
      RVG0707V100A&lt;BR&gt;
      RVA0607V(H)306A&lt;BR&gt;
      RVA1214H213A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      3104B&lt;BR&gt;
      3104C&lt;BR&gt;
      3104F&lt;BR&gt;
      3104H&lt;BR&gt;
      -&lt;BR&gt;
      3104M&lt;BR&gt;
      3104P&lt;BR&gt;
      3104S&lt;BR&gt;
      3104W&lt;BR&gt;
      3104X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      EVMQ0G&lt;BR&gt;
      EVMQIG&lt;BR&gt;
      EVMQ3G&lt;BR&gt;
      EVMS0G&lt;BR&gt;
      EVMQ0G&lt;BR&gt;
      EVMG0G&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMK4GA00B&lt;BR&gt;
      EVM30GA00B&lt;BR&gt;
      EVMK0GA00B&lt;BR&gt;
      EVM38GA00B&lt;BR&gt;
      EVMB6&lt;BR&gt;
      EVLQ0&lt;BR&gt;
      -&lt;BR&gt;
      EVMMSG&lt;BR&gt;
      EVMMBG&lt;BR&gt;
      EVMMAG&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMMCS&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM0&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      EVMM3&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      62-3-1&lt;BR&gt;
      62-1-2&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67R&lt;BR&gt;
      -&lt;BR&gt;
      67P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      67X&lt;BR&gt;
      63V&lt;BR&gt;
      63S&lt;BR&gt;
      63M&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63H&lt;BR&gt;
      63P&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      63X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      RJ/RJR50&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;&amp;nbsp;&lt;P&gt;
&lt;TABLE BORDER=0 CELLSPACING=1 CELLPADDING=3&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT color="#0000FF" SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SMD TRIM-POT CROSS REFERENCE&lt;/B&gt;&lt;/FONT&gt;
      &lt;P&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;MULTI-TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3224G&lt;BR&gt;
      3224J&lt;BR&gt;
      3224W&lt;BR&gt;
      3269P&lt;BR&gt;
      3269W&lt;BR&gt;
      3269X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      44G&lt;BR&gt;
      44J&lt;BR&gt;
      44W&lt;BR&gt;
      84P&lt;BR&gt;
      84W&lt;BR&gt;
      84X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST63Z&lt;BR&gt;
      ST63Y&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST5P&lt;BR&gt;
      ST5W&lt;BR&gt;
      ST5X&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;
      &lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;SINGLE TURN&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BOURNS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;BI&amp;nbsp;TECH&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;DALE-VISHAY&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PHILIPS/MEPCO&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;PANASONIC&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;TOCOS&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD&gt;
      &lt;FONT SIZE=3 FACE=ARIAL&gt;&lt;B&gt;AUX/KYOCERA&lt;/B&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
  &lt;TR&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      3314G&lt;BR&gt;
      3314J&lt;BR&gt;
      3364A/B&lt;BR&gt;
      3364C/D&lt;BR&gt;
      3364W/X&lt;BR&gt;
      3313G&lt;BR&gt;
      3313J&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      23B&lt;BR&gt;
      23A&lt;BR&gt;
      21X&lt;BR&gt;
      21W&lt;BR&gt;
      -&lt;BR&gt;
      22B&lt;BR&gt;
      22A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST5YL/ST53YL&lt;BR&gt;
      ST5YJ/5T53YJ&lt;BR&gt;
      ST-23A&lt;BR&gt;
      ST-22B&lt;BR&gt;
      ST-22&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      ST-4B&lt;BR&gt;
      ST-4A&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      ST-3B&lt;BR&gt;
      ST-3A&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      EVM-6YS&lt;BR&gt;
      EVM-1E&lt;BR&gt;
      EVM-1G&lt;BR&gt;
      EVM-1D&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      G4B&lt;BR&gt;
      G4A&lt;BR&gt;
      TR04-3S1&lt;BR&gt;
      TRG04-2S1&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;
      DVR-43A&lt;BR&gt;
      CVR-42C&lt;BR&gt;
      CVR-42A/C&lt;BR&gt;
      -&lt;BR&gt;
      -&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;
&lt;/TABLE&gt;
&lt;P&gt;
&lt;FONT SIZE=4 FACE=ARIAL&gt;&lt;B&gt;ALT =&amp;nbsp;ALTERNATE&lt;/B&gt;&lt;/FONT&gt;
&lt;P&gt;

&amp;nbsp;
&lt;P&gt;
&lt;/td&gt;
&lt;/tr&gt;
&lt;/table&gt;</description>
<packages>
<package name="R0402" urn="urn:adsk.eagle:footprint:23043/3" library_version="11">
<description>&lt;b&gt;Chip RESISTOR 0402 EIA (1005 Metric)&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1" y1="0.483" x2="1" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1" y1="0.483" x2="1" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1" y1="-0.483" x2="-1" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1" y1="-0.483" x2="-1" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.5" y="0" dx="0.6" dy="0.7" layer="1"/>
<smd name="2" x="0.5" y="0" dx="0.6" dy="0.7" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.35" x2="0.1999" y2="0.35" layer="35"/>
</package>
<package name="R0603" urn="urn:adsk.eagle:footprint:23044/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.432" y1="-0.356" x2="0.432" y2="-0.356" width="0.1524" layer="51"/>
<wire x1="0.432" y1="0.356" x2="-0.432" y2="0.356" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1" dy="1.1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1" dy="1.1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4318" y1="-0.4318" x2="0.8382" y2="0.4318" layer="51"/>
<rectangle x1="-0.8382" y1="-0.4318" x2="-0.4318" y2="0.4318" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="R0805" urn="urn:adsk.eagle:footprint:23045/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R0805W" urn="urn:adsk.eagle:footprint:23046/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; wave soldering&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.0525" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="1.0525" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="R1206" urn="urn:adsk.eagle:footprint:23047/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="0.9525" y1="-0.8128" x2="-0.9652" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="0.9525" y1="0.8128" x2="-0.9652" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="2" x="1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<smd name="1" x="-1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6891" y1="-0.8763" x2="-0.9525" y2="0.8763" layer="51"/>
<rectangle x1="0.9525" y1="-0.8763" x2="1.6891" y2="0.8763" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1206W" urn="urn:adsk.eagle:footprint:23048/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R1210" urn="urn:adsk.eagle:footprint:23049/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8999" x2="0.3" y2="0.8999" layer="35"/>
</package>
<package name="R1210W" urn="urn:adsk.eagle:footprint:23050/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="R2010" urn="urn:adsk.eagle:footprint:23051/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2010W" urn="urn:adsk.eagle:footprint:23052/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
</package>
<package name="R2012" urn="urn:adsk.eagle:footprint:23053/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2012W" urn="urn:adsk.eagle:footprint:23054/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.94" y="0" dx="1.5" dy="1" layer="1"/>
<smd name="2" x="0.94" y="0" dx="1.5" dy="1" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
</package>
<package name="R2512" urn="urn:adsk.eagle:footprint:23055/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<smd name="2" x="2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R2512W" urn="urn:adsk.eagle:footprint:23056/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-2.896" y="0" dx="2" dy="2.1" layer="1"/>
<smd name="2" x="2.896" y="0" dx="2" dy="2.1" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R3216" urn="urn:adsk.eagle:footprint:23057/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3216W" urn="urn:adsk.eagle:footprint:23058/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.2" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="R3225" urn="urn:adsk.eagle:footprint:23059/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R3225W" urn="urn:adsk.eagle:footprint:23060/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<smd name="2" x="1.499" y="0" dx="1.8" dy="1.8" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
</package>
<package name="R5025" urn="urn:adsk.eagle:footprint:23061/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R5025W" urn="urn:adsk.eagle:footprint:23062/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
wave soldering</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<smd name="1" x="-2.311" y="0" dx="2" dy="1.8" layer="1"/>
<smd name="2" x="2.311" y="0" dx="2" dy="1.8" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332" urn="urn:adsk.eagle:footprint:23063/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-3.1" y="0" dx="1" dy="3.2" layer="1"/>
<smd name="2" x="3.1" y="0" dx="1" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="R6332W" urn="urn:adsk.eagle:footprint:25646/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; wave soldering&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<smd name="1" x="-3.196" y="0" dx="1.2" dy="3.2" layer="1"/>
<smd name="2" x="3.196" y="0" dx="1.2" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M0805" urn="urn:adsk.eagle:footprint:23065/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M1206" urn="urn:adsk.eagle:footprint:23066/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M1406" urn="urn:adsk.eagle:footprint:23067/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
</package>
<package name="M2012" urn="urn:adsk.eagle:footprint:23068/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.10 W</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="0.7112" y1="0.635" x2="-0.7112" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.7112" y1="-0.635" x2="-0.7112" y2="-0.635" width="0.1524" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0414" y1="-0.7112" x2="-0.6858" y2="0.7112" layer="51"/>
<rectangle x1="0.6858" y1="-0.7112" x2="1.0414" y2="0.7112" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5999" x2="0.1999" y2="0.5999" layer="35"/>
</package>
<package name="M2309" urn="urn:adsk.eagle:footprint:23069/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="M3216" urn="urn:adsk.eagle:footprint:23070/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="1.143" y1="0.8382" x2="-1.143" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.8382" x2="-1.143" y2="-0.8382" width="0.1524" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.9144" x2="-1.1176" y2="0.9144" layer="51"/>
<rectangle x1="1.1176" y1="-0.9144" x2="1.7018" y2="0.9144" layer="51"/>
<rectangle x1="-0.3" y1="-0.8001" x2="0.3" y2="0.8001" layer="35"/>
</package>
<package name="M3516" urn="urn:adsk.eagle:footprint:23071/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.12 W</description>
<wire x1="-2.973" y1="0.983" x2="2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-0.983" x2="-2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-0.983" x2="-2.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="0.983" x2="2.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.3208" y1="0.762" x2="-1.3208" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.3208" y1="-0.762" x2="-1.3208" y2="-0.762" width="0.1524" layer="51"/>
<smd name="1" x="-1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<smd name="2" x="1.7" y="0" dx="1.4" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.8542" y1="-0.8382" x2="-1.2954" y2="0.8382" layer="51"/>
<rectangle x1="1.2954" y1="-0.8382" x2="1.8542" y2="0.8382" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="M5923" urn="urn:adsk.eagle:footprint:23072/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
MELF 0.25 W</description>
<wire x1="-4.473" y1="1.483" x2="4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="-1.483" x2="-4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-4.473" y1="-1.483" x2="-4.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="4.473" y1="1.483" x2="4.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.413" y1="1.1684" x2="-2.4384" y2="1.1684" width="0.1524" layer="51"/>
<wire x1="2.413" y1="-1.1684" x2="-2.413" y2="-1.1684" width="0.1524" layer="51"/>
<smd name="1" x="-2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<smd name="2" x="2.85" y="0" dx="1.5" dy="2.6" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.048" y1="-1.2446" x2="-2.3876" y2="1.2446" layer="51"/>
<rectangle x1="2.3876" y1="-1.2446" x2="3.048" y2="1.2446" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="0204/5" urn="urn:adsk.eagle:footprint:22991/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0" x2="-2.032" y2="0" width="0.508" layer="51"/>
<wire x1="-1.778" y1="0.635" x2="-1.524" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.524" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="-0.889" x2="1.778" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="1.524" y1="0.889" x2="1.778" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.778" y1="-0.635" x2="-1.778" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="0.889" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="0.762" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-0.889" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="-0.762" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="0.762" x2="-1.143" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="-1.143" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="1.524" y1="0.889" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.524" y1="-0.889" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.635" x2="1.778" y2="0.635" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.0066" y="1.1684" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.032" y1="-0.254" x2="-1.778" y2="0.254" layer="51"/>
<rectangle x1="1.778" y1="-0.254" x2="2.032" y2="0.254" layer="51"/>
</package>
<package name="0204/7" urn="urn:adsk.eagle:footprint:22998/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 7.5 mm</description>
<wire x1="3.81" y1="0" x2="2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-3.81" y1="0" x2="-2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0.762" x2="-2.286" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.286" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="-0.762" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.016" x2="2.54" y2="0.762" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="1.016" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="0.889" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.016" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-0.889" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="-1.778" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="-1.778" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="2.286" y1="1.016" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.286" y1="-1.016" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="2.54" y2="0.762" width="0.1524" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.2954" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.6256" y="-0.4826" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.54" y1="-0.254" x2="2.921" y2="0.254" layer="21"/>
<rectangle x1="-2.921" y1="-0.254" x2="-2.54" y2="0.254" layer="21"/>
</package>
<package name="0207/10" urn="urn:adsk.eagle:footprint:22992/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 10 mm</description>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.048" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.2606" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
</package>
<package name="0207/12" urn="urn:adsk.eagle:footprint:22993/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 12 mm</description>
<wire x1="6.35" y1="0" x2="5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.334" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="4.445" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-4.445" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.3086" y2="0.3048" layer="21"/>
<rectangle x1="-5.3086" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
</package>
<package name="0207/15" urn="urn:adsk.eagle:footprint:22997/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 15mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="5.715" y1="0" x2="4.064" y2="0" width="0.6096" layer="21"/>
<wire x1="-5.715" y1="0" x2="-4.064" y2="0" width="0.6096" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.175" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<rectangle x1="5.715" y1="-0.3048" x2="6.5786" y2="0.3048" layer="21"/>
<rectangle x1="-6.5786" y1="-0.3048" x2="-5.715" y2="0.3048" layer="21"/>
</package>
<package name="0207/2V" urn="urn:adsk.eagle:footprint:22994/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="-0.381" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.254" y1="0" x2="0.254" y2="0" width="0.6096" layer="21"/>
<wire x1="0.381" y1="0" x2="1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.27" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-0.0508" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.0508" y="-2.2352" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/5V" urn="urn:adsk.eagle:footprint:22995/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-0.889" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.762" y1="0" x2="0.762" y2="0" width="0.6096" layer="21"/>
<wire x1="0.889" y1="0" x2="2.54" y2="0" width="0.6096" layer="51"/>
<circle x="-2.54" y="0" radius="1.27" width="0.1016" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.143" y="0.889" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.143" y="-2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0207/7" urn="urn:adsk.eagle:footprint:22996/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 7.5 mm</description>
<wire x1="-3.81" y1="0" x2="-3.429" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="51"/>
<wire x1="3.429" y1="0" x2="3.81" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-0.5588" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-3.429" y1="-0.3048" x2="-3.175" y2="0.3048" layer="51"/>
<rectangle x1="3.175" y1="-0.3048" x2="3.429" y2="0.3048" layer="51"/>
</package>
<package name="0309/10" urn="urn:adsk.eagle:footprint:23073/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 10mm</description>
<wire x1="-4.699" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="51"/>
<wire x1="5.08" y1="0" x2="4.699" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.6228" y1="-0.3048" x2="-4.318" y2="0.3048" layer="51"/>
<rectangle x1="4.318" y1="-0.3048" x2="4.6228" y2="0.3048" layer="51"/>
</package>
<package name="0309/12" urn="urn:adsk.eagle:footprint:23074/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="-4.318" y1="1.27" x2="-4.064" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.064" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="-1.524" x2="4.318" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="4.064" y1="1.524" x2="4.318" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.318" y1="-1.27" x2="-4.318" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="1.397" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.524" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="-1.397" x2="-3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="1.397" x2="-3.302" y2="1.397" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.302" y1="-1.397" x2="-3.302" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.524" x2="3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.318" y1="-1.27" x2="4.318" y2="1.27" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" shape="octagon"/>
<text x="-4.191" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.6858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.318" y1="-0.3048" x2="5.1816" y2="0.3048" layer="21"/>
<rectangle x1="-5.1816" y1="-0.3048" x2="-4.318" y2="0.3048" layer="21"/>
</package>
<package name="0411/12" urn="urn:adsk.eagle:footprint:23076/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 12.5 mm</description>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.762" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.762" layer="51"/>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<pad name="1" x="-6.35" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.3594" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
<rectangle x1="5.08" y1="-0.381" x2="5.3594" y2="0.381" layer="21"/>
</package>
<package name="0411/15" urn="urn:adsk.eagle:footprint:23077/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 15 mm</description>
<wire x1="5.08" y1="-1.651" x2="5.08" y2="1.651" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.032" x2="5.08" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.651" x2="-4.699" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="4.699" y1="-2.032" x2="5.08" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.651" x2="-4.699" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="2.032" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="1.905" x2="4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.032" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="3.937" y1="-1.905" x2="4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="1.905" x2="3.937" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.937" y1="-1.905" x2="3.937" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.651" x2="-5.08" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="2.032" x2="-4.064" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-2.032" x2="-4.064" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0" x2="-6.35" y2="0" width="0.762" layer="51"/>
<wire x1="6.35" y1="0" x2="7.62" y2="0" width="0.762" layer="51"/>
<pad name="1" x="-7.62" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.9144" shape="octagon"/>
<text x="-5.08" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.5814" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="5.08" y1="-0.381" x2="6.477" y2="0.381" layer="21"/>
<rectangle x1="-6.477" y1="-0.381" x2="-5.08" y2="0.381" layer="21"/>
</package>
<package name="0411V" urn="urn:adsk.eagle:footprint:23078/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0411, grid 3.81 mm</description>
<wire x1="1.27" y1="0" x2="0.3048" y2="0" width="0.762" layer="51"/>
<wire x1="-1.5748" y1="0" x2="-2.54" y2="0" width="0.762" layer="51"/>
<circle x="-2.54" y="0" radius="2.032" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.016" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.9144" shape="octagon"/>
<text x="-0.508" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.5334" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.4732" y1="-0.381" x2="0.2032" y2="0.381" layer="21"/>
</package>
<package name="0414/15" urn="urn:adsk.eagle:footprint:23079/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.604" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="1.905" x2="-5.842" y2="2.159" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-5.842" y2="-2.159" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="-2.159" x2="6.096" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="5.842" y1="2.159" x2="6.096" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.096" y1="-1.905" x2="-6.096" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="2.159" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="2.032" x2="-4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-5.842" y1="-2.159" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="-4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-2.032" x2="-4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.842" y1="2.159" x2="4.953" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.842" y1="-2.159" x2="4.953" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-1.905" x2="6.096" y2="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.5654" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="6.096" y1="-0.4064" x2="6.5024" y2="0.4064" layer="21"/>
<rectangle x1="-6.5024" y1="-0.4064" x2="-6.096" y2="0.4064" layer="21"/>
</package>
<package name="0414V" urn="urn:adsk.eagle:footprint:23080/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0414, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.159" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.381" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.381" y="-2.3622" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.2954" y2="0.4064" layer="21"/>
</package>
<package name="0617/17" urn="urn:adsk.eagle:footprint:23081/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 17.5 mm</description>
<wire x1="-8.89" y1="0" x2="-8.636" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-8.255" y1="1.016" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="8.255" y1="1.016" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="8.636" y1="0" x2="8.89" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-8.89" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.016" shape="octagon"/>
<text x="-8.128" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.096" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-8.5344" y1="-0.4064" x2="-8.2296" y2="0.4064" layer="51"/>
<rectangle x1="8.2296" y1="-0.4064" x2="8.5344" y2="0.4064" layer="51"/>
</package>
<package name="0617/22" urn="urn:adsk.eagle:footprint:23082/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 22.5 mm</description>
<wire x1="-10.287" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="-2.667" x2="-8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="3.048" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="2.794" x2="-6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="-3.048" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="-2.794" x2="-6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="2.794" x2="-6.731" y2="2.794" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.794" x2="-6.731" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="7.874" y1="3.048" x2="6.985" y2="3.048" width="0.1524" layer="21"/>
<wire x1="7.874" y1="-3.048" x2="6.985" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.255" y1="-2.667" x2="8.255" y2="2.667" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.287" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.255" y1="2.667" x2="-7.874" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.255" y1="-2.667" x2="-7.874" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="7.874" y1="3.048" x2="8.255" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="7.874" y1="-3.048" x2="8.255" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.255" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.1854" y1="-0.4064" x2="-8.255" y2="0.4064" layer="21"/>
<rectangle x1="8.255" y1="-0.4064" x2="10.1854" y2="0.4064" layer="21"/>
</package>
<package name="0617V" urn="urn:adsk.eagle:footprint:23083/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0617, grid 5 mm</description>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="3.048" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="0.635" y="1.4224" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.635" y="-2.6162" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.3208" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="0922/22" urn="urn:adsk.eagle:footprint:23084/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 22.5 mm</description>
<wire x1="11.43" y1="0" x2="10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-11.43" y1="0" x2="-10.795" y2="0" width="0.8128" layer="51"/>
<wire x1="-10.16" y1="-4.191" x2="-10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="4.572" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="4.318" x2="-8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-9.779" y1="-4.572" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-8.636" y1="-4.318" x2="-8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="4.318" x2="-8.636" y2="4.318" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="8.636" y1="-4.318" x2="-8.636" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="9.779" y1="4.572" x2="8.89" y2="4.572" width="0.1524" layer="21"/>
<wire x1="9.779" y1="-4.572" x2="8.89" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="10.16" y1="-4.191" x2="10.16" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-4.191" x2="-9.779" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-10.16" y1="4.191" x2="-9.779" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="9.779" y1="-4.572" x2="10.16" y2="-4.191" width="0.1524" layer="21" curve="90"/>
<wire x1="9.779" y1="4.572" x2="10.16" y2="4.191" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-10.16" y="5.1054" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.477" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.7188" y1="-0.4064" x2="-10.16" y2="0.4064" layer="51"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-10.16" y2="0.4064" layer="21"/>
<rectangle x1="10.16" y1="-0.4064" x2="10.7188" y2="0.4064" layer="51"/>
</package>
<package name="P0613V" urn="urn:adsk.eagle:footprint:23085/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.8128" layer="51"/>
<circle x="-2.54" y="0" radius="2.286" width="0.1524" layer="21"/>
<circle x="-2.54" y="0" radius="1.143" width="0.1524" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.254" y="1.143" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.254" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.2954" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="P0613/15" urn="urn:adsk.eagle:footprint:23086/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0613, grid 15 mm</description>
<wire x1="7.62" y1="0" x2="6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.985" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.032" x2="-6.223" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.032" x2="-6.223" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="-2.286" x2="6.477" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="6.223" y1="2.286" x2="6.477" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.223" y1="2.286" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.159" x2="-5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-6.223" y1="-2.286" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-2.159" x2="-5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="2.159" x2="-5.207" y2="2.159" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.159" x2="-5.207" y2="-2.159" width="0.1524" layer="21"/>
<wire x1="6.223" y1="2.286" x2="5.334" y2="2.286" width="0.1524" layer="21"/>
<wire x1="6.223" y1="-2.286" x2="5.334" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-0.635" x2="6.477" y2="0.635" width="0.1524" layer="51"/>
<wire x1="6.477" y1="2.032" x2="6.477" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.032" x2="-6.477" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="0.635" x2="-6.477" y2="2.032" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.477" y="2.6924" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.7112" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-7.0358" y1="-0.4064" x2="-6.477" y2="0.4064" layer="51"/>
<rectangle x1="6.477" y1="-0.4064" x2="7.0358" y2="0.4064" layer="51"/>
</package>
<package name="P0817/22" urn="urn:adsk.eagle:footprint:23087/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 22.5 mm</description>
<wire x1="-10.414" y1="0" x2="-11.43" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="-3.429" x2="-8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="3.81" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="3.556" x2="-7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-8.128" y1="-3.81" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-3.556" x2="-7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="3.556" x2="-6.985" y2="3.556" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-3.556" x2="-6.985" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.128" y1="3.81" x2="7.239" y2="3.81" width="0.1524" layer="21"/>
<wire x1="8.128" y1="-3.81" x2="7.239" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.429" x2="8.509" y2="3.429" width="0.1524" layer="21"/>
<wire x1="11.43" y1="0" x2="10.414" y2="0" width="0.8128" layer="51"/>
<wire x1="-8.509" y1="3.429" x2="-8.128" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="-8.509" y1="-3.429" x2="-8.128" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="8.128" y1="3.81" x2="8.509" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.128" y1="-3.81" x2="8.509" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-11.43" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.43" y="0" drill="1.016" shape="octagon"/>
<text x="-8.382" y="4.2164" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.223" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="6.604" y="-2.2606" size="1.27" layer="51" ratio="10" rot="R90">0817</text>
<rectangle x1="8.509" y1="-0.4064" x2="10.3124" y2="0.4064" layer="21"/>
<rectangle x1="-10.3124" y1="-0.4064" x2="-8.509" y2="0.4064" layer="21"/>
</package>
<package name="P0817V" urn="urn:adsk.eagle:footprint:23088/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0817, grid 6.35 mm</description>
<wire x1="-3.81" y1="0" x2="-5.08" y2="0" width="0.8128" layer="51"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="3.81" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.27" width="0.1524" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="1.016" shape="octagon"/>
<text x="-1.016" y="1.27" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.016" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.032" size="1.016" layer="21" ratio="12">0817</text>
<rectangle x1="-3.81" y1="-0.4064" x2="0" y2="0.4064" layer="21"/>
</package>
<package name="V234/12" urn="urn:adsk.eagle:footprint:23089/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V234, grid 12.5 mm</description>
<wire x1="-4.953" y1="1.524" x2="-4.699" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="1.778" x2="4.953" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.778" x2="4.953" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.953" y1="-1.524" x2="-4.699" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="1.778" x2="4.699" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="1.524" x2="-4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.778" x2="-4.699" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="4.953" y1="1.524" x2="4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.35" y1="0" x2="5.461" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.461" y2="0" width="0.8128" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.016" shape="octagon"/>
<text x="-4.953" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="4.953" y1="-0.4064" x2="5.4102" y2="0.4064" layer="21"/>
<rectangle x1="-5.4102" y1="-0.4064" x2="-4.953" y2="0.4064" layer="21"/>
</package>
<package name="V235/17" urn="urn:adsk.eagle:footprint:23090/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V235, grid 17.78 mm</description>
<wire x1="-6.731" y1="2.921" x2="6.731" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-7.112" y1="2.54" x2="-7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.731" y1="-2.921" x2="-6.731" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="7.112" y1="2.54" x2="7.112" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.89" y1="0" x2="7.874" y2="0" width="1.016" layer="51"/>
<wire x1="-7.874" y1="0" x2="-8.89" y2="0" width="1.016" layer="51"/>
<wire x1="-7.112" y1="-2.54" x2="-6.731" y2="-2.921" width="0.1524" layer="21" curve="90"/>
<wire x1="6.731" y1="2.921" x2="7.112" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.731" y1="-2.921" x2="7.112" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-7.112" y1="2.54" x2="-6.731" y2="2.921" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-8.89" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="8.89" y="0" drill="1.1938" shape="octagon"/>
<text x="-6.858" y="3.302" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.842" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="7.112" y1="-0.508" x2="7.747" y2="0.508" layer="21"/>
<rectangle x1="-7.747" y1="-0.508" x2="-7.112" y2="0.508" layer="21"/>
</package>
<package name="V526-0" urn="urn:adsk.eagle:footprint:23091/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type V526-0, grid 2.5 mm</description>
<wire x1="-2.54" y1="1.016" x2="-2.286" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="1.27" x2="2.54" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.286" y1="-1.27" x2="2.54" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.54" y1="-1.016" x2="-2.286" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.27" x2="-2.286" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.016" x2="2.54" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.27" x2="2.286" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.54" y2="-1.016" width="0.1524" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.413" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.413" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102AX" urn="urn:adsk.eagle:footprint:23100/1" library_version="11">
<description>&lt;b&gt;Mini MELF 0102 Axial&lt;/b&gt;</description>
<circle x="0" y="0" radius="0.6" width="0" layer="51"/>
<circle x="0" y="0" radius="0.6" width="0" layer="52"/>
<smd name="1" x="0" y="0" dx="1.9" dy="1.9" layer="1" roundness="100"/>
<smd name="2" x="0" y="0" dx="1.9" dy="1.9" layer="16" roundness="100"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
<hole x="0" y="0" drill="1.3"/>
</package>
<package name="0922V" urn="urn:adsk.eagle:footprint:23098/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0922, grid 7.5 mm</description>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.8128" layer="51"/>
<wire x1="-5.08" y1="0" x2="-3.81" y2="0" width="0.8128" layer="51"/>
<circle x="-5.08" y="0" radius="4.572" width="0.1524" layer="21"/>
<circle x="-5.08" y="0" radius="1.905" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="1.016" shape="octagon"/>
<text x="-0.508" y="1.6764" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.508" y="-2.9972" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.858" y="2.54" size="1.016" layer="21" ratio="12">0922</text>
<rectangle x1="-3.81" y1="-0.4064" x2="1.3208" y2="0.4064" layer="21"/>
</package>
<package name="MINI_MELF-0102R" urn="urn:adsk.eagle:footprint:23092/1" library_version="11">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<smd name="2" x="0.9" y="0" dx="0.5" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0102W" urn="urn:adsk.eagle:footprint:23093/1" library_version="11">
<description>&lt;b&gt;CECC Size RC2211&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1" y1="-0.5" x2="1" y2="-0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="-0.5" x2="1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="1" y1="0.5" x2="-1" y2="0.5" width="0.2032" layer="51"/>
<wire x1="-1" y1="0.5" x2="-1" y2="-0.5" width="0.2032" layer="51"/>
<smd name="1" x="-0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<smd name="2" x="0.95" y="0" dx="0.6" dy="1.3" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204R" urn="urn:adsk.eagle:footprint:25676/1" library_version="11">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="0.938" y1="0.6" x2="-0.938" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-0.938" y1="-0.6" x2="0.938" y2="-0.6" width="0.2032" layer="21"/>
<smd name="1" x="-1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="0.8" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0204W" urn="urn:adsk.eagle:footprint:25677/1" library_version="11">
<description>&lt;b&gt;CECC Size RC3715&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-1.7" y1="-0.6" x2="1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="-0.6" x2="1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="1.7" y1="0.6" x2="-1.7" y2="0.6" width="0.2032" layer="51"/>
<wire x1="-1.7" y1="0.6" x2="-1.7" y2="-0.6" width="0.2032" layer="51"/>
<wire x1="0.684" y1="0.6" x2="-0.684" y2="0.6" width="0.2032" layer="21"/>
<wire x1="-0.684" y1="-0.6" x2="0.684" y2="-0.6" width="0.2032" layer="21"/>
<smd name="1" x="-1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.2" dy="1.6" layer="1"/>
<text x="-1.27" y="0.9525" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.2225" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207R" urn="urn:adsk.eagle:footprint:25678/1" library_version="11">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Reflow Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.2125" y1="1" x2="-1.2125" y2="1" width="0.2032" layer="21"/>
<wire x1="-1.2125" y1="-1" x2="1.2125" y2="-1" width="0.2032" layer="21"/>
<smd name="1" x="-2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<smd name="2" x="2.25" y="0" dx="1.6" dy="2.5" layer="1"/>
<text x="-2.2225" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.2225" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="MINI_MELF-0207W" urn="urn:adsk.eagle:footprint:25679/1" library_version="11">
<description>&lt;b&gt;CECC Size RC6123&lt;/b&gt; Wave Soldering&lt;p&gt;
source Beyschlag</description>
<wire x1="-2.8" y1="-1" x2="2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="-1" x2="2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="2.8" y1="1" x2="-2.8" y2="1" width="0.2032" layer="51"/>
<wire x1="-2.8" y1="1" x2="-2.8" y2="-1" width="0.2032" layer="51"/>
<wire x1="1.149" y1="1" x2="-1.149" y2="1" width="0.2032" layer="21"/>
<wire x1="-1.149" y1="-1" x2="1.149" y2="-1" width="0.2032" layer="21"/>
<smd name="1" x="-2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<smd name="2" x="2.6" y="0" dx="2.4" dy="2.5" layer="1"/>
<text x="-2.54" y="1.5875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="RDH/15" urn="urn:adsk.eagle:footprint:23099/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type RDH, grid 15 mm</description>
<wire x1="-7.62" y1="0" x2="-6.858" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.096" y1="3.048" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="2.794" x2="-5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="-3.048" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.794" x2="-5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.794" x2="-4.953" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="4.953" y1="-2.794" x2="-4.953" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="5.207" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="5.207" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="-2.667" x2="-6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-6.477" y1="1.016" x2="-6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.477" y1="-2.667" x2="6.477" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="6.477" y1="1.016" x2="6.477" y2="2.667" width="0.1524" layer="21"/>
<wire x1="6.858" y1="0" x2="7.62" y2="0" width="0.8128" layer="51"/>
<wire x1="-6.477" y1="2.667" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="3.048" x2="6.477" y2="2.667" width="0.1524" layer="21" curve="-90"/>
<wire x1="-6.477" y1="-2.667" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="6.096" y1="-3.048" x2="6.477" y2="-2.667" width="0.1524" layer="21" curve="90"/>
<pad name="1" x="-7.62" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="1.016" shape="octagon"/>
<text x="-6.35" y="3.4544" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-0.5842" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="4.572" y="-1.7272" size="1.27" layer="51" ratio="10" rot="R90">RDH</text>
<rectangle x1="-6.7564" y1="-0.4064" x2="-6.4516" y2="0.4064" layer="51"/>
<rectangle x1="6.4516" y1="-0.4064" x2="6.7564" y2="0.4064" layer="51"/>
</package>
<package name="0204V" urn="urn:adsk.eagle:footprint:22999/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 2.5 mm</description>
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.508" layer="51"/>
<wire x1="-0.127" y1="0" x2="0.127" y2="0" width="0.508" layer="21"/>
<circle x="-1.27" y="0" radius="0.889" width="0.1524" layer="51"/>
<circle x="-1.27" y="0" radius="0.635" width="0.0508" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.1336" y="1.1684" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.1336" y="-2.3114" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0309V" urn="urn:adsk.eagle:footprint:23075/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0309, grid 2.5 mm</description>
<wire x1="1.27" y1="0" x2="0.635" y2="0" width="0.6096" layer="51"/>
<wire x1="-0.635" y1="0" x2="-1.27" y2="0" width="0.6096" layer="51"/>
<circle x="-1.27" y="0" radius="1.524" width="0.1524" layer="21"/>
<circle x="-1.27" y="0" radius="0.762" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="0.254" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0.254" y="-2.2098" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="0.254" y1="-0.3048" x2="0.5588" y2="0.3048" layer="51"/>
<rectangle x1="-0.635" y1="-0.3048" x2="-0.3302" y2="0.3048" layer="51"/>
<rectangle x1="-0.3302" y1="-0.3048" x2="0.254" y2="0.3048" layer="21"/>
</package>
<package name="R0201" urn="urn:adsk.eagle:footprint:25683/1" library_version="11">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; chip&lt;p&gt;
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<smd name="1" x="-0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<smd name="2" x="0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.3" y1="-0.15" x2="-0.15" y2="0.15" layer="51"/>
<rectangle x1="0.15" y1="-0.15" x2="0.3" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="-0.15" x2="0.15" y2="0.15" layer="21"/>
</package>
<package name="VMTA55" urn="urn:adsk.eagle:footprint:25689/1" library_version="11">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RNC55&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-5.08" y1="0" x2="-4.26" y2="0" width="0.6096" layer="51"/>
<wire x1="3.3375" y1="-1.45" x2="3.3375" y2="1.45" width="0.1524" layer="21"/>
<wire x1="3.3375" y1="1.45" x2="-3.3625" y2="1.45" width="0.1524" layer="21"/>
<wire x1="-3.3625" y1="1.45" x2="-3.3625" y2="-1.45" width="0.1524" layer="21"/>
<wire x1="-3.3625" y1="-1.45" x2="3.3375" y2="-1.45" width="0.1524" layer="21"/>
<wire x1="4.235" y1="0" x2="5.08" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.1" shape="octagon"/>
<text x="-3.175" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-4.26" y1="-0.3048" x2="-3.3075" y2="0.3048" layer="21"/>
<rectangle x1="3.2825" y1="-0.3048" x2="4.235" y2="0.3048" layer="21"/>
</package>
<package name="VMTB60" urn="urn:adsk.eagle:footprint:25690/1" library_version="11">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RNC60&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-6.35" y1="0" x2="-5.585" y2="0" width="0.6096" layer="51"/>
<wire x1="4.6875" y1="-1.95" x2="4.6875" y2="1.95" width="0.1524" layer="21"/>
<wire x1="4.6875" y1="1.95" x2="-4.6875" y2="1.95" width="0.1524" layer="21"/>
<wire x1="-4.6875" y1="1.95" x2="-4.6875" y2="-1.95" width="0.1524" layer="21"/>
<wire x1="-4.6875" y1="-1.95" x2="4.6875" y2="-1.95" width="0.1524" layer="21"/>
<wire x1="5.585" y1="0" x2="6.35" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.1" shape="octagon"/>
<text x="-4.445" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.445" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.585" y1="-0.3048" x2="-4.6325" y2="0.3048" layer="21"/>
<rectangle x1="4.6325" y1="-0.3048" x2="5.585" y2="0.3048" layer="21"/>
</package>
<package name="VTA52" urn="urn:adsk.eagle:footprint:25684/1" library_version="11">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR52&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-15.24" y1="0" x2="-13.97" y2="0" width="0.6096" layer="51"/>
<wire x1="12.6225" y1="0.025" x2="12.6225" y2="4.725" width="0.1524" layer="21"/>
<wire x1="12.6225" y1="4.725" x2="-12.6225" y2="4.725" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="4.725" x2="-12.6225" y2="0.025" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="0.025" x2="-12.6225" y2="-4.65" width="0.1524" layer="21"/>
<wire x1="-12.6225" y1="-4.65" x2="12.6225" y2="-4.65" width="0.1524" layer="21"/>
<wire x1="12.6225" y1="-4.65" x2="12.6225" y2="0.025" width="0.1524" layer="21"/>
<wire x1="13.97" y1="0" x2="15.24" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-15.24" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="15.24" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-13.97" y1="-0.3048" x2="-12.5675" y2="0.3048" layer="21"/>
<rectangle x1="12.5675" y1="-0.3048" x2="13.97" y2="0.3048" layer="21"/>
</package>
<package name="VTA53" urn="urn:adsk.eagle:footprint:25685/1" library_version="11">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR53&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-12.065" y1="0" x2="-10.795" y2="0" width="0.6096" layer="51"/>
<wire x1="9.8975" y1="0" x2="9.8975" y2="4.7" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="4.7" x2="-9.8975" y2="4.7" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="4.7" x2="-9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="0" x2="-9.8975" y2="-4.675" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="-4.675" x2="9.8975" y2="-4.675" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="-4.675" x2="9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="10.795" y1="0" x2="12.065" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.065" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="12.065" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.795" y1="-0.3048" x2="-9.8425" y2="0.3048" layer="21"/>
<rectangle x1="9.8425" y1="-0.3048" x2="10.795" y2="0.3048" layer="21"/>
</package>
<package name="VTA54" urn="urn:adsk.eagle:footprint:25686/1" library_version="11">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR54&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-12.065" y1="0" x2="-10.795" y2="0" width="0.6096" layer="51"/>
<wire x1="9.8975" y1="0" x2="9.8975" y2="3.3" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="3.3" x2="-9.8975" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="3.3" x2="-9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="0" x2="-9.8975" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-9.8975" y1="-3.3" x2="9.8975" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="9.8975" y1="-3.3" x2="9.8975" y2="0" width="0.1524" layer="21"/>
<wire x1="10.795" y1="0" x2="12.065" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.065" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="12.065" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-10.795" y1="-0.3048" x2="-9.8425" y2="0.3048" layer="21"/>
<rectangle x1="9.8425" y1="-0.3048" x2="10.795" y2="0.3048" layer="21"/>
</package>
<package name="VTA55" urn="urn:adsk.eagle:footprint:25687/1" library_version="11">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR55&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-8.255" y1="0" x2="-6.985" y2="0" width="0.6096" layer="51"/>
<wire x1="6.405" y1="0" x2="6.405" y2="3.3" width="0.1524" layer="21"/>
<wire x1="6.405" y1="3.3" x2="-6.405" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="3.3" x2="-6.405" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="0" x2="-6.405" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-6.405" y1="-3.3" x2="6.405" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="6.405" y1="-3.3" x2="6.405" y2="0" width="0.1524" layer="21"/>
<wire x1="6.985" y1="0" x2="8.255" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-8.255" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="8.255" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-6.985" y1="-0.3048" x2="-6.35" y2="0.3048" layer="21"/>
<rectangle x1="6.35" y1="-0.3048" x2="6.985" y2="0.3048" layer="21"/>
</package>
<package name="VTA56" urn="urn:adsk.eagle:footprint:25688/1" library_version="11">
<description>&lt;b&gt;Bulk Metal® Foil Technology&lt;/b&gt;, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements&lt;p&gt;
MIL SIZE RBR56&lt;br&gt;
Source: VISHAY .. vta56.pdf</description>
<wire x1="-6.35" y1="0" x2="-5.08" y2="0" width="0.6096" layer="51"/>
<wire x1="4.5" y1="0" x2="4.5" y2="3.3" width="0.1524" layer="21"/>
<wire x1="4.5" y1="3.3" x2="-4.5" y2="3.3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="3.3" x2="-4.5" y2="0" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="0" x2="-4.5" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="-4.5" y1="-3.3" x2="4.5" y2="-3.3" width="0.1524" layer="21"/>
<wire x1="4.5" y1="-3.3" x2="4.5" y2="0" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0" x2="6.35" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="1.1" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="1.1" shape="octagon"/>
<text x="-3.81" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.08" y1="-0.3048" x2="-4.445" y2="0.3048" layer="21"/>
<rectangle x1="4.445" y1="-0.3048" x2="5.08" y2="0.3048" layer="21"/>
</package>
<package name="R4527" urn="urn:adsk.eagle:footprint:13246/1" library_version="11">
<description>&lt;b&gt;Package 4527&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<wire x1="-5.675" y1="-3.375" x2="5.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.65" y1="-3.375" x2="5.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.65" y1="3.375" x2="-5.675" y2="3.375" width="0.2032" layer="21"/>
<wire x1="-5.675" y1="3.375" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<smd name="1" x="-4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.715" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.715" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC0001" urn="urn:adsk.eagle:footprint:25692/1" library_version="11">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3.075" y1="-1.8" x2="3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="-1.8" x2="3.075" y2="1.8" width="0.2032" layer="51"/>
<wire x1="3.075" y1="1.8" x2="-3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="1.606" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="-1.606" x2="-3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="1.606" x2="3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="3.075" y1="-1.8" x2="3.075" y2="-1.606" width="0.2032" layer="21"/>
<smd name="1" x="-2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<smd name="2" x="2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<text x="-2.544" y="2.229" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.544" y="-3.501" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC0002" urn="urn:adsk.eagle:footprint:25693/1" library_version="11">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-5.55" y1="3.375" x2="-5.55" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-5.55" y1="-3.375" x2="5.55" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.55" y1="-3.375" x2="5.55" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.55" y1="3.375" x2="-5.55" y2="3.375" width="0.2032" layer="21"/>
<smd name="1" x="-4.575" y="0.025" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.65" y="3.9" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.65" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC01/2" urn="urn:adsk.eagle:footprint:25694/1" library_version="11">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-2.45" y1="1.475" x2="-2.45" y2="-1.475" width="0.2032" layer="51"/>
<wire x1="-2.45" y1="-1.475" x2="2.45" y2="-1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="-1.475" x2="2.45" y2="1.475" width="0.2032" layer="51"/>
<wire x1="2.45" y1="1.475" x2="-2.45" y2="1.475" width="0.2032" layer="21"/>
<wire x1="-2.45" y1="1.475" x2="-2.45" y2="1.106" width="0.2032" layer="21"/>
<wire x1="-2.45" y1="-1.106" x2="-2.45" y2="-1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="1.106" x2="2.45" y2="1.475" width="0.2032" layer="21"/>
<wire x1="2.45" y1="-1.475" x2="2.45" y2="-1.106" width="0.2032" layer="21"/>
<smd name="1" x="-2.1" y="0" dx="2.16" dy="1.78" layer="1"/>
<smd name="2" x="2.1" y="0" dx="2.16" dy="1.78" layer="1"/>
<text x="-2.544" y="1.904" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.544" y="-3.176" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC2515" urn="urn:adsk.eagle:footprint:25695/1" library_version="11">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="-1.8" width="0.2032" layer="51"/>
<wire x1="-3.075" y1="-1.8" x2="3.05" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="-1.8" x2="3.05" y2="1.8" width="0.2032" layer="51"/>
<wire x1="3.05" y1="1.8" x2="-3.075" y2="1.8" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="1.8" x2="-3.075" y2="1.606" width="0.2032" layer="21"/>
<wire x1="-3.075" y1="-1.606" x2="-3.075" y2="-1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="1.606" x2="3.05" y2="1.8" width="0.2032" layer="21"/>
<wire x1="3.05" y1="-1.8" x2="3.05" y2="-1.606" width="0.2032" layer="21"/>
<smd name="1" x="-2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<smd name="2" x="2.675" y="0" dx="2.29" dy="2.92" layer="1"/>
<text x="-3.2" y="2.15" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.2" y="-3.4" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC4527" urn="urn:adsk.eagle:footprint:25696/1" library_version="11">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-5.675" y1="3.4" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-5.675" y1="-3.375" x2="5.675" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.675" y1="-3.375" x2="5.675" y2="3.4" width="0.2032" layer="51"/>
<wire x1="5.675" y1="3.4" x2="-5.675" y2="3.4" width="0.2032" layer="21"/>
<smd name="1" x="-4.575" y="0.025" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.775" y="3.925" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.775" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="WSC6927" urn="urn:adsk.eagle:footprint:25697/1" library_version="11">
<description>&lt;b&gt;Wirewound Resistors, Precision Power&lt;/b&gt;&lt;p&gt;
Source: VISHAY wscwsn.pdf</description>
<wire x1="-8.65" y1="3.375" x2="-8.65" y2="-3.375" width="0.2032" layer="51"/>
<wire x1="-8.65" y1="-3.375" x2="8.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="8.65" y1="-3.375" x2="8.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="8.65" y1="3.375" x2="-8.65" y2="3.375" width="0.2032" layer="21"/>
<smd name="1" x="-7.95" y="0.025" dx="3.94" dy="5.97" layer="1"/>
<smd name="2" x="7.95" y="0" dx="3.94" dy="5.97" layer="1"/>
<text x="-8.75" y="3.9" size="1.27" layer="25">&gt;NAME</text>
<text x="-8.75" y="-5.15" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R1218" urn="urn:adsk.eagle:footprint:25698/1" library_version="11">
<description>&lt;b&gt;CRCW1218 Thick Film, Rectangular Chip Resistors&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com .. dcrcw.pdf</description>
<wire x1="-0.913" y1="-2.219" x2="0.939" y2="-2.219" width="0.1524" layer="51"/>
<wire x1="0.913" y1="2.219" x2="-0.939" y2="2.219" width="0.1524" layer="51"/>
<smd name="1" x="-1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<smd name="2" x="1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<text x="-2.54" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-2.3" x2="-0.9009" y2="2.3" layer="51"/>
<rectangle x1="0.9144" y1="-2.3" x2="1.6645" y2="2.3" layer="51"/>
</package>
<package name="1812X7R" urn="urn:adsk.eagle:footprint:25699/1" library_version="11">
<description>&lt;b&gt;Chip Monolithic Ceramic Capacitors&lt;/b&gt; Medium Voltage High Capacitance for General Use&lt;p&gt;
Source: http://www.murata.com .. GRM43DR72E224KW01.pdf</description>
<wire x1="-1.1" y1="1.5" x2="1.1" y2="1.5" width="0.2032" layer="51"/>
<wire x1="1.1" y1="-1.5" x2="-1.1" y2="-1.5" width="0.2032" layer="51"/>
<wire x1="-0.6" y1="1.5" x2="0.6" y2="1.5" width="0.2032" layer="21"/>
<wire x1="0.6" y1="-1.5" x2="-0.6" y2="-1.5" width="0.2032" layer="21"/>
<smd name="1" x="-1.425" y="0" dx="0.8" dy="3.5" layer="1"/>
<smd name="2" x="1.425" y="0" dx="0.8" dy="3.5" layer="1" rot="R180"/>
<text x="-1.9456" y="1.9958" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.9456" y="-3.7738" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.4" y1="-1.6" x2="-1.1" y2="1.6" layer="51"/>
<rectangle x1="1.1" y1="-1.6" x2="1.4" y2="1.6" layer="51" rot="R180"/>
</package>
<package name="R01005" urn="urn:adsk.eagle:footprint:25701/1" library_version="11">
<smd name="1" x="-0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<smd name="2" x="0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<text x="-0.4" y="0.3" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.4" y="-1.6" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.2" y1="-0.1" x2="-0.075" y2="0.1" layer="51"/>
<rectangle x1="0.075" y1="-0.1" x2="0.2" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="0.05" x2="0.15" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="-0.1" x2="0.15" y2="-0.05" layer="51"/>
</package>
<package name="C0402" urn="urn:adsk.eagle:footprint:23121/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C0504" urn="urn:adsk.eagle:footprint:23122/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.294" y1="0.559" x2="0.294" y2="0.559" width="0.1016" layer="51"/>
<wire x1="-0.294" y1="-0.559" x2="0.294" y2="-0.559" width="0.1016" layer="51"/>
<smd name="1" x="-0.7" y="0" dx="1" dy="1.3" layer="1"/>
<smd name="2" x="0.7" y="0" dx="1" dy="1.3" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.6604" y1="-0.6223" x2="-0.2804" y2="0.6276" layer="51"/>
<rectangle x1="0.2794" y1="-0.6223" x2="0.6594" y2="0.6276" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C0603" urn="urn:adsk.eagle:footprint:23123/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.1" dy="1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.1" dy="1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8382" y1="-0.4699" x2="-0.3381" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C0805" urn="urn:adsk.eagle:footprint:23124/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0922" y1="-0.7239" x2="-0.3421" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C1206" urn="urn:adsk.eagle:footprint:23125/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.8509" x2="-0.9517" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="C1210" urn="urn:adsk.eagle:footprint:23126/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="-0.9652" y1="1.2446" x2="0.9652" y2="1.2446" width="0.1016" layer="51"/>
<wire x1="-0.9652" y1="-1.2446" x2="0.9652" y2="-1.2446" width="0.1016" layer="51"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-1.2954" x2="-0.9517" y2="1.3045" layer="51"/>
<rectangle x1="0.9517" y1="-1.3045" x2="1.7018" y2="1.2954" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
</package>
<package name="C1310" urn="urn:adsk.eagle:footprint:23127/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.294" y1="0.559" x2="0.294" y2="0.559" width="0.1016" layer="51"/>
<wire x1="-0.294" y1="-0.559" x2="0.294" y2="-0.559" width="0.1016" layer="51"/>
<smd name="1" x="-0.7" y="0" dx="1" dy="1.3" layer="1"/>
<smd name="2" x="0.7" y="0" dx="1" dy="1.3" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.6604" y1="-0.6223" x2="-0.2804" y2="0.6276" layer="51"/>
<rectangle x1="0.2794" y1="-0.6223" x2="0.6594" y2="0.6276" layer="51"/>
<rectangle x1="-0.1001" y1="-0.3" x2="0.1001" y2="0.3" layer="35"/>
</package>
<package name="C1608" urn="urn:adsk.eagle:footprint:23128/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.356" y1="0.432" x2="0.356" y2="0.432" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.419" x2="0.356" y2="-0.419" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.1" dy="1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.1" dy="1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8382" y1="-0.4699" x2="-0.3381" y2="0.4801" layer="51"/>
<rectangle x1="0.3302" y1="-0.4699" x2="0.8303" y2="0.4801" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
</package>
<package name="C1812" urn="urn:adsk.eagle:footprint:23129/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="1.983" x2="2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-1.983" x2="-2.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-1.983" x2="-2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="-1.4732" y1="1.6002" x2="1.4732" y2="1.6002" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-1.6002" x2="1.4732" y2="-1.6002" width="0.1016" layer="51"/>
<wire x1="2.973" y1="1.983" x2="2.973" y2="-1.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<text x="-1.905" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.3876" y1="-1.651" x2="-1.4376" y2="1.649" layer="51"/>
<rectangle x1="1.4478" y1="-1.651" x2="2.3978" y2="1.649" layer="51"/>
<rectangle x1="-0.3" y1="-0.4001" x2="0.3" y2="0.4001" layer="35"/>
</package>
<package name="C1825" urn="urn:adsk.eagle:footprint:23130/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="3.483" x2="2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-3.483" x2="-2.973" y2="-3.483" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-3.483" x2="-2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="-1.4986" y1="3.2766" x2="1.4732" y2="3.2766" width="0.1016" layer="51"/>
<wire x1="-1.4732" y1="-3.2766" x2="1.4986" y2="-3.2766" width="0.1016" layer="51"/>
<wire x1="2.973" y1="3.483" x2="2.973" y2="-3.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<text x="-1.905" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.413" y1="-3.3528" x2="-1.463" y2="3.3472" layer="51"/>
<rectangle x1="1.4478" y1="-3.3528" x2="2.3978" y2="3.3472" layer="51"/>
<rectangle x1="-0.7" y1="-0.7" x2="0.7" y2="0.7" layer="35"/>
</package>
<package name="C2012" urn="urn:adsk.eagle:footprint:23131/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.0922" y1="-0.7239" x2="-0.3421" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1001" y1="-0.4001" x2="0.1001" y2="0.4001" layer="35"/>
</package>
<package name="C3216" urn="urn:adsk.eagle:footprint:23132/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-0.965" y1="0.787" x2="0.965" y2="0.787" width="0.1016" layer="51"/>
<wire x1="-0.965" y1="-0.787" x2="0.965" y2="-0.787" width="0.1016" layer="51"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-0.8509" x2="-0.9517" y2="0.8491" layer="51"/>
<rectangle x1="0.9517" y1="-0.8491" x2="1.7018" y2="0.8509" layer="51"/>
<rectangle x1="-0.3" y1="-0.5001" x2="0.3" y2="0.5001" layer="35"/>
</package>
<package name="C3225" urn="urn:adsk.eagle:footprint:23133/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="-0.9652" y1="1.2446" x2="0.9652" y2="1.2446" width="0.1016" layer="51"/>
<wire x1="-0.9652" y1="-1.2446" x2="0.9652" y2="-1.2446" width="0.1016" layer="51"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7018" y1="-1.2954" x2="-0.9517" y2="1.3045" layer="51"/>
<rectangle x1="0.9517" y1="-1.3045" x2="1.7018" y2="1.2954" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
</package>
<package name="C4532" urn="urn:adsk.eagle:footprint:23134/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="1.983" x2="2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-1.983" x2="-2.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-1.983" x2="-2.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="-1.4732" y1="1.6002" x2="1.4732" y2="1.6002" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-1.6002" x2="1.4732" y2="-1.6002" width="0.1016" layer="51"/>
<wire x1="2.973" y1="1.983" x2="2.973" y2="-1.983" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="3.4" layer="1"/>
<text x="-1.905" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.3876" y1="-1.651" x2="-1.4376" y2="1.649" layer="51"/>
<rectangle x1="1.4478" y1="-1.651" x2="2.3978" y2="1.649" layer="51"/>
<rectangle x1="-0.4001" y1="-0.7" x2="0.4001" y2="0.7" layer="35"/>
</package>
<package name="C4564" urn="urn:adsk.eagle:footprint:23135/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<wire x1="-2.973" y1="3.483" x2="2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="2.973" y1="-3.483" x2="-2.973" y2="-3.483" width="0.0508" layer="39"/>
<wire x1="-2.973" y1="-3.483" x2="-2.973" y2="3.483" width="0.0508" layer="39"/>
<wire x1="-1.4986" y1="3.2766" x2="1.4732" y2="3.2766" width="0.1016" layer="51"/>
<wire x1="-1.4732" y1="-3.2766" x2="1.4986" y2="-3.2766" width="0.1016" layer="51"/>
<wire x1="2.973" y1="3.483" x2="2.973" y2="-3.483" width="0.0508" layer="39"/>
<smd name="1" x="-1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.9" dy="6.8" layer="1"/>
<text x="-1.905" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.413" y1="-3.3528" x2="-1.463" y2="3.3472" layer="51"/>
<rectangle x1="1.4478" y1="-3.3528" x2="2.3978" y2="3.3472" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
</package>
<package name="C025-024X044" urn="urn:adsk.eagle:footprint:23136/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 2.4 x 4.4 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.778" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.778" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-025X050" urn="urn:adsk.eagle:footprint:23137/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 2.5 x 5 mm</description>
<wire x1="-2.159" y1="1.27" x2="2.159" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.27" x2="-2.159" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.27" x2="2.413" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.016" x2="-2.159" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.27" x2="2.413" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.016" x2="-2.159" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-030X050" urn="urn:adsk.eagle:footprint:23138/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 3 x 5 mm</description>
<wire x1="-2.159" y1="1.524" x2="2.159" y2="1.524" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.524" x2="-2.159" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.27" x2="2.413" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.27" x2="-2.413" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.524" x2="2.413" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.27" x2="-2.159" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.524" x2="2.413" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.27" x2="-2.159" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-040X050" urn="urn:adsk.eagle:footprint:23139/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 4 x 5 mm</description>
<wire x1="-2.159" y1="1.905" x2="2.159" y2="1.905" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.905" x2="-2.159" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.651" x2="2.413" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.651" x2="-2.413" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.905" x2="2.413" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.651" x2="-2.159" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.905" x2="2.413" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.651" x2="-2.159" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.429" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-050X050" urn="urn:adsk.eagle:footprint:23140/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 5 x 5 mm</description>
<wire x1="-2.159" y1="2.286" x2="2.159" y2="2.286" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.286" x2="-2.159" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.032" x2="2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.032" x2="-2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.286" x2="2.413" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.032" x2="-2.159" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.286" x2="2.413" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.032" x2="-2.159" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025-060X050" urn="urn:adsk.eagle:footprint:23141/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm, outline 6 x 5 mm</description>
<wire x1="-2.159" y1="2.794" x2="2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.794" x2="-2.159" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.54" x2="2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.54" x2="-2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.794" x2="2.413" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.54" x2="-2.159" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.794" x2="2.413" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.54" x2="-2.159" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-024X070" urn="urn:adsk.eagle:footprint:23142/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 mm + 5 mm, outline 2.4 x 7 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.191" y1="-1.143" x2="-3.9624" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-4.191" y1="1.143" x2="-3.9624" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-4.699" y1="-0.635" x2="-4.191" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="0.635" x2="-4.191" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.699" y1="-0.635" x2="-4.699" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="1.143" x2="-2.5654" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.143" x2="-2.5654" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.81" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-025X075" urn="urn:adsk.eagle:footprint:23143/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 2.5 x 7.5 mm</description>
<wire x1="-2.159" y1="1.27" x2="2.159" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.27" x2="-2.159" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.27" x2="2.413" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.016" x2="-2.159" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.27" x2="2.413" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.016" x2="-2.159" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="1.016" x2="4.953" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="4.699" y1="1.27" x2="4.953" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.27" x2="4.953" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="1.27" x2="4.699" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.27" x2="2.794" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.413" y2="0.762" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-0.762" x2="2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.254" x2="2.413" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0" x2="2.286" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.762" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.159" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.159" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-035X075" urn="urn:adsk.eagle:footprint:23144/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 3.5 x 7.5 mm</description>
<wire x1="-2.159" y1="1.778" x2="2.159" y2="1.778" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.778" x2="-2.159" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.524" x2="-2.413" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.778" x2="2.413" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.524" x2="-2.159" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.778" x2="2.413" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.524" x2="-2.159" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="1.524" x2="4.953" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="4.699" y1="1.778" x2="4.953" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-1.778" x2="4.953" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="1.778" x2="4.699" y2="1.778" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.778" x2="2.794" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.524" x2="2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.413" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.508" x2="2.413" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="1.778" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-045X075" urn="urn:adsk.eagle:footprint:23145/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 4.5 x 7.5 mm</description>
<wire x1="-2.159" y1="2.286" x2="2.159" y2="2.286" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.286" x2="-2.159" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.032" x2="-2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.286" x2="2.413" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.032" x2="-2.159" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.286" x2="2.413" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.032" x2="-2.159" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="2.032" x2="4.953" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.286" x2="4.953" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-2.286" x2="4.953" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="2.286" x2="4.699" y2="2.286" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.286" x2="2.794" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.032" x2="2.413" y2="1.397" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.397" x2="2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.762" x2="2.413" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="1.778" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="2.667" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.286" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C025_050-055X075" urn="urn:adsk.eagle:footprint:23146/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 2.5 + 5 mm, outline 5.5 x 7.5 mm</description>
<wire x1="-2.159" y1="2.794" x2="2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.794" x2="-2.159" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.54" x2="-2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.794" x2="2.413" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.54" x2="-2.159" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.794" x2="2.413" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.54" x2="-2.159" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-0.762" y2="0" width="0.1524" layer="51"/>
<wire x1="4.953" y1="2.54" x2="4.953" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="4.699" y1="2.794" x2="4.953" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="-2.794" x2="4.953" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="2.794" y1="2.794" x2="4.699" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-2.794" x2="2.794" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.54" x2="2.413" y2="2.032" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-2.032" x2="2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.413" y1="0.762" x2="2.413" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0" x2="2.286" y2="0" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0" x2="2.794" y2="0" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0" x2="3.302" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.762" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="3.81" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.286" y="3.175" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-024X044" urn="urn:adsk.eagle:footprint:23147/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 2.4 x 4.4 mm</description>
<wire x1="-2.159" y1="-0.635" x2="-2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-2.159" y1="0.635" x2="-1.651" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.159" y1="-0.635" x2="-1.651" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="1.651" y1="1.143" x2="-1.651" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-0.635" x2="2.159" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-1.143" x2="-1.651" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="1.651" y1="1.143" x2="2.159" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="1.651" y1="-1.143" x2="2.159" y2="-0.635" width="0.1524" layer="21" curve="90"/>
<wire x1="-0.3048" y1="0.762" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0.762" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.762" width="0.3048" layer="21"/>
<wire x1="1.27" y1="0" x2="0.3302" y2="0" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0" x2="-0.3048" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.159" y="1.397" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.159" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.159" y1="-0.381" x2="2.54" y2="0.381" layer="51"/>
<rectangle x1="-2.54" y1="-0.381" x2="-2.159" y2="0.381" layer="51"/>
</package>
<package name="C050-025X075" urn="urn:adsk.eagle:footprint:23148/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 2.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.016" x2="-3.683" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.27" x2="3.429" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.016" x2="3.683" y2="1.016" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.27" x2="-3.429" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.27" x2="3.683" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.27" x2="3.683" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.016" x2="-3.429" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.016" x2="-3.429" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-045X075" urn="urn:adsk.eagle:footprint:23149/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 4.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.032" x2="-3.683" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.286" x2="3.429" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.032" x2="3.683" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.286" x2="-3.429" y2="2.286" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.286" x2="3.683" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.286" x2="3.683" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.032" x2="-3.429" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.032" x2="-3.429" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="2.667" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-030X075" urn="urn:adsk.eagle:footprint:23150/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 3 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.27" x2="-3.683" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.27" x2="3.683" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.524" x2="3.683" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.524" x2="3.683" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.27" x2="-3.429" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.27" x2="-3.429" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-050X075" urn="urn:adsk.eagle:footprint:23151/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.286" x2="-3.683" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.54" x2="3.429" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.286" x2="3.683" y2="2.286" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.54" x2="-3.429" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.54" x2="3.683" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.54" x2="3.683" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.286" x2="-3.429" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.286" x2="-3.429" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-055X075" urn="urn:adsk.eagle:footprint:23152/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 5.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.54" x2="-3.683" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.794" x2="3.429" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.54" x2="3.683" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.794" x2="-3.429" y2="2.794" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.794" x2="3.683" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.794" x2="3.683" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.54" x2="-3.429" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.54" x2="-3.429" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="3.175" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.302" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-075X075" urn="urn:adsk.eagle:footprint:23153/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 7.5 x 7.5 mm</description>
<wire x1="-1.524" y1="0" x2="-0.4572" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.4572" y1="0" x2="-0.4572" y2="0.762" width="0.4064" layer="21"/>
<wire x1="-0.4572" y1="0" x2="-0.4572" y2="-0.762" width="0.4064" layer="21"/>
<wire x1="0.4318" y1="0.762" x2="0.4318" y2="0" width="0.4064" layer="21"/>
<wire x1="0.4318" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.4318" y1="0" x2="0.4318" y2="-0.762" width="0.4064" layer="21"/>
<wire x1="-3.683" y1="3.429" x2="-3.683" y2="-3.429" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-3.683" x2="3.429" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-3.429" x2="3.683" y2="3.429" width="0.1524" layer="21"/>
<wire x1="3.429" y1="3.683" x2="-3.429" y2="3.683" width="0.1524" layer="21"/>
<wire x1="3.429" y1="3.683" x2="3.683" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-3.683" x2="3.683" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-3.429" x2="-3.429" y2="-3.683" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="3.429" x2="-3.429" y2="3.683" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.429" y="4.064" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050H075X075" urn="urn:adsk.eagle:footprint:23154/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Horizontal, grid 5 mm, outline 7.5 x 7.5 mm</description>
<wire x1="-3.683" y1="7.112" x2="-3.683" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="0.508" x2="-3.302" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-3.302" y1="0.508" x2="-1.778" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="0.508" x2="1.778" y2="0.508" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.508" x2="3.302" y2="0.508" width="0.1524" layer="51"/>
<wire x1="3.302" y1="0.508" x2="3.683" y2="0.508" width="0.1524" layer="21"/>
<wire x1="3.683" y1="0.508" x2="3.683" y2="7.112" width="0.1524" layer="21"/>
<wire x1="3.175" y1="7.62" x2="-3.175" y2="7.62" width="0.1524" layer="21"/>
<wire x1="-0.3048" y1="2.413" x2="-0.3048" y2="1.778" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="1.778" x2="-0.3048" y2="1.143" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="1.778" x2="-1.651" y2="1.778" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="2.413" x2="0.3302" y2="1.778" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="1.778" x2="0.3302" y2="1.143" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="1.778" x2="1.651" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="7.112" x2="-3.175" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.175" y1="7.62" x2="3.683" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="0" x2="-2.54" y2="0.254" width="0.508" layer="51"/>
<wire x1="2.54" y1="0" x2="2.54" y2="0.254" width="0.508" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.302" y="8.001" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="3.175" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-2.794" y1="0.127" x2="-2.286" y2="0.508" layer="51"/>
<rectangle x1="2.286" y1="0.127" x2="2.794" y2="0.508" layer="51"/>
</package>
<package name="C075-032X103" urn="urn:adsk.eagle:footprint:23155/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 3.2 x 10.3 mm</description>
<wire x1="4.826" y1="1.524" x2="-4.826" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-1.524" x2="4.826" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.27" x2="5.08" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.826" y1="1.524" x2="5.08" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.826" y1="-1.524" x2="5.08" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="-1.27" x2="-4.826" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.27" x2="-4.826" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="0.508" y1="0" x2="2.54" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0" x2="-0.508" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.508" y1="0.889" x2="-0.508" y2="0" width="0.4064" layer="21"/>
<wire x1="-0.508" y1="0" x2="-0.508" y2="-0.889" width="0.4064" layer="21"/>
<wire x1="0.508" y1="0.889" x2="0.508" y2="0" width="0.4064" layer="21"/>
<wire x1="0.508" y1="0" x2="0.508" y2="-0.889" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="1.905" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.826" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-042X103" urn="urn:adsk.eagle:footprint:23156/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 4.2 x 10.3 mm</description>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.778" x2="-5.08" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.778" x2="5.08" y2="1.778" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="5.08" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.826" y1="-2.032" x2="5.08" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="-1.778" x2="-4.826" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.778" x2="-4.826" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.699" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-052X106" urn="urn:adsk.eagle:footprint:23157/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 5.2 x 10.6 mm</description>
<wire x1="4.953" y1="2.54" x2="-4.953" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.286" x2="-5.207" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.54" x2="4.953" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.286" x2="5.207" y2="2.286" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.54" x2="5.207" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.953" y1="-2.54" x2="5.207" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="-2.286" x2="-4.953" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="2.286" x2="-4.953" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-043X133" urn="urn:adsk.eagle:footprint:23158/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 4.3 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="2.032" x2="6.096" y2="2.032" width="0.1524" layer="21"/>
<wire x1="6.604" y1="1.524" x2="6.604" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-2.032" x2="-6.096" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-1.524" x2="-6.604" y2="1.524" width="0.1524" layer="21"/>
<wire x1="6.096" y1="2.032" x2="6.604" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-2.032" x2="6.604" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-1.524" x2="-6.096" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="1.524" x2="-6.096" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-054X133" urn="urn:adsk.eagle:footprint:23159/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 5.4 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="2.54" x2="6.096" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.604" y1="2.032" x2="6.604" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-2.54" x2="-6.096" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.032" x2="-6.604" y2="2.032" width="0.1524" layer="21"/>
<wire x1="6.096" y1="2.54" x2="6.604" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-2.54" x2="6.604" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.032" x2="-6.096" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.032" x2="-6.096" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-1.905" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102-064X133" urn="urn:adsk.eagle:footprint:23160/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm, outline 6.4 x 13.3 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="3.048" x2="6.096" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.604" y1="2.54" x2="6.604" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="-6.096" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.54" x2="-6.604" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="6.604" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-3.048" x2="6.604" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.54" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.54" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<text x="-6.096" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C102_152-062X184" urn="urn:adsk.eagle:footprint:23161/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 10.2 mm + 15.2 mm, outline 6.2 x 18.4 mm</description>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.683" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="3.683" y2="0" width="0.1524" layer="21"/>
<wire x1="6.477" y1="0" x2="8.636" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="3.048" x2="6.223" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.223" y1="-3.048" x2="-6.096" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.54" x2="-6.604" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.223" y1="3.048" x2="6.731" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.223" y1="-3.048" x2="6.731" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.54" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.54" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.731" y1="2.54" x2="6.731" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="11.176" y1="3.048" x2="11.684" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="11.176" y1="-3.048" x2="11.684" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="11.176" y1="-3.048" x2="7.112" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="7.112" y1="3.048" x2="11.176" y2="3.048" width="0.1524" layer="21"/>
<wire x1="11.684" y1="2.54" x2="11.684" y2="-2.54" width="0.1524" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" shape="octagon"/>
<pad name="3" x="10.033" y="0" drill="1.016" shape="octagon"/>
<text x="-5.969" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-054X183" urn="urn:adsk.eagle:footprint:23162/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 5.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="2.032" x2="9.017" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-2.54" x2="-8.509" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-2.032" x2="-9.017" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="2.54" x2="8.509" y2="2.54" width="0.1524" layer="21"/>
<wire x1="8.509" y1="2.54" x2="9.017" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-2.54" x2="9.017" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-2.032" x2="-8.509" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="2.032" x2="-8.509" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.382" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-064X183" urn="urn:adsk.eagle:footprint:23163/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 6.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="2.54" x2="9.017" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.048" x2="-8.509" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-2.54" x2="-9.017" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="3.048" x2="8.509" y2="3.048" width="0.1524" layer="21"/>
<wire x1="8.509" y1="3.048" x2="9.017" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-3.048" x2="9.017" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-2.54" x2="-8.509" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="2.54" x2="-8.509" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-072X183" urn="urn:adsk.eagle:footprint:23164/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 7.2 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.048" x2="9.017" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.556" x2="-8.509" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.048" x2="-9.017" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="3.556" x2="8.509" y2="3.556" width="0.1524" layer="21"/>
<wire x1="8.509" y1="3.556" x2="9.017" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-3.556" x2="9.017" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.048" x2="-8.509" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.048" x2="-8.509" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="3.937" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-084X183" urn="urn:adsk.eagle:footprint:23165/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 8.4 x 18.3 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.556" x2="9.017" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-4.064" x2="-8.509" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.556" x2="-9.017" y2="3.556" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="4.064" x2="8.509" y2="4.064" width="0.1524" layer="21"/>
<wire x1="8.509" y1="4.064" x2="9.017" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-4.064" x2="9.017" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.556" x2="-8.509" y2="-4.064" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.556" x2="-8.509" y2="4.064" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="4.445" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C150-091X182" urn="urn:adsk.eagle:footprint:23166/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 15 mm, outline 9.1 x 18.2 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.937" x2="9.017" y2="-3.937" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-4.445" x2="-8.509" y2="-4.445" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.937" x2="-9.017" y2="3.937" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="4.445" x2="8.509" y2="4.445" width="0.1524" layer="21"/>
<wire x1="8.509" y1="4.445" x2="9.017" y2="3.937" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-4.445" x2="9.017" y2="-3.937" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.937" x2="-8.509" y2="-4.445" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.937" x2="-8.509" y2="4.445" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" shape="octagon"/>
<text x="-8.509" y="4.826" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-062X268" urn="urn:adsk.eagle:footprint:23167/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 6.2 x 26.8 mm</description>
<wire x1="-12.827" y1="3.048" x2="12.827" y2="3.048" width="0.1524" layer="21"/>
<wire x1="13.335" y1="2.54" x2="13.335" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-3.048" x2="-12.827" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-2.54" x2="-13.335" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="3.048" x2="13.335" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-3.048" x2="13.335" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-2.54" x2="-12.827" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="2.54" x2="-12.827" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.7" y="3.429" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-074X268" urn="urn:adsk.eagle:footprint:23168/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 7.4 x 26.8 mm</description>
<wire x1="-12.827" y1="3.556" x2="12.827" y2="3.556" width="0.1524" layer="21"/>
<wire x1="13.335" y1="3.048" x2="13.335" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-3.556" x2="-12.827" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-3.048" x2="-13.335" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="3.556" x2="13.335" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-3.556" x2="13.335" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-3.048" x2="-12.827" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="3.048" x2="-12.827" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.827" y="3.937" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-087X268" urn="urn:adsk.eagle:footprint:23169/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 8.7 x 26.8 mm</description>
<wire x1="-12.827" y1="4.318" x2="12.827" y2="4.318" width="0.1524" layer="21"/>
<wire x1="13.335" y1="3.81" x2="13.335" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-4.318" x2="-12.827" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-3.81" x2="-13.335" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="4.318" x2="13.335" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-4.318" x2="13.335" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-3.81" x2="-12.827" y2="-4.318" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="3.81" x2="-12.827" y2="4.318" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.827" y="4.699" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-108X268" urn="urn:adsk.eagle:footprint:23170/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 10.8 x 26.8 mm</description>
<wire x1="-12.827" y1="5.334" x2="12.827" y2="5.334" width="0.1524" layer="21"/>
<wire x1="13.335" y1="4.826" x2="13.335" y2="-4.826" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-5.334" x2="-12.827" y2="-5.334" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-4.826" x2="-13.335" y2="4.826" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="5.334" x2="13.335" y2="4.826" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-5.334" x2="13.335" y2="-4.826" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-4.826" x2="-12.827" y2="-5.334" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="4.826" x2="-12.827" y2="5.334" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.954" y="5.715" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C225-113X268" urn="urn:adsk.eagle:footprint:23171/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 22.5 mm, outline 11.3 x 26.8 mm</description>
<wire x1="-12.827" y1="5.588" x2="12.827" y2="5.588" width="0.1524" layer="21"/>
<wire x1="13.335" y1="5.08" x2="13.335" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-5.588" x2="-12.827" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-5.08" x2="-13.335" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="5.588" x2="13.335" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-5.588" x2="13.335" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-5.08" x2="-12.827" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="5.08" x2="-12.827" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" shape="octagon"/>
<text x="-12.954" y="5.969" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-093X316" urn="urn:adsk.eagle:footprint:23172/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 9.3 x 31.6 mm</description>
<wire x1="-15.24" y1="4.572" x2="15.24" y2="4.572" width="0.1524" layer="21"/>
<wire x1="15.748" y1="4.064" x2="15.748" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-4.572" x2="-15.24" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-4.064" x2="-15.748" y2="4.064" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="4.572" x2="15.748" y2="4.064" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-4.572" x2="15.748" y2="-4.064" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-4.064" x2="-15.24" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="4.064" x2="-15.24" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="4.953" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-113X316" urn="urn:adsk.eagle:footprint:23173/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 11.3 x 31.6 mm</description>
<wire x1="-15.24" y1="5.588" x2="15.24" y2="5.588" width="0.1524" layer="21"/>
<wire x1="15.748" y1="5.08" x2="15.748" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-5.588" x2="-15.24" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-5.08" x2="-15.748" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="5.588" x2="15.748" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-5.588" x2="15.748" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-5.08" x2="-15.24" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="5.08" x2="-15.24" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="5.969" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-134X316" urn="urn:adsk.eagle:footprint:23174/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 13.4 x 31.6 mm</description>
<wire x1="-15.24" y1="6.604" x2="15.24" y2="6.604" width="0.1524" layer="21"/>
<wire x1="15.748" y1="6.096" x2="15.748" y2="-6.096" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-6.604" x2="-15.24" y2="-6.604" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-6.096" x2="-15.748" y2="6.096" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="6.604" x2="15.748" y2="6.096" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-6.604" x2="15.748" y2="-6.096" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-6.096" x2="-15.24" y2="-6.604" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="6.096" x2="-15.24" y2="6.604" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="6.985" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-205X316" urn="urn:adsk.eagle:footprint:23175/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 20.5 x 31.6 mm</description>
<wire x1="-15.24" y1="10.16" x2="15.24" y2="10.16" width="0.1524" layer="21"/>
<wire x1="15.748" y1="9.652" x2="15.748" y2="-9.652" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-10.16" x2="-15.24" y2="-10.16" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-9.652" x2="-15.748" y2="9.652" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="10.16" x2="15.748" y2="9.652" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-10.16" x2="15.748" y2="-9.652" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-9.652" x2="-15.24" y2="-10.16" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="9.652" x2="-15.24" y2="10.16" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="10.541" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-4.318" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-137X374" urn="urn:adsk.eagle:footprint:23176/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 13.7 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="6.731" x2="18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="6.731" x2="-18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-6.731" x2="18.542" y2="-6.731" width="0.1524" layer="21"/>
<wire x1="18.542" y1="6.731" x2="-18.542" y2="6.731" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.2372" y="7.0612" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-162X374" urn="urn:adsk.eagle:footprint:23177/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 16.2 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="8.001" x2="18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="8.001" x2="-18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-8.001" x2="18.542" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="18.542" y1="8.001" x2="-18.542" y2="8.001" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.3642" y="8.3312" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C325-182X374" urn="urn:adsk.eagle:footprint:23178/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 32.5 mm, outline 18.2 x 37.4 mm</description>
<wire x1="-14.2748" y1="0" x2="-12.7" y2="0" width="0.1524" layer="21"/>
<wire x1="-12.7" y1="1.905" x2="-12.7" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="1.905" x2="-11.811" y2="0" width="0.4064" layer="21"/>
<wire x1="-11.811" y1="0" x2="14.2748" y2="0" width="0.1524" layer="21"/>
<wire x1="-11.811" y1="0" x2="-11.811" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-12.7" y1="0" x2="-12.7" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="18.542" y1="9.017" x2="18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="9.017" x2="-18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="-9.017" x2="18.542" y2="-9.017" width="0.1524" layer="21"/>
<wire x1="18.542" y1="9.017" x2="-18.542" y2="9.017" width="0.1524" layer="21"/>
<pad name="1" x="-16.256" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="16.256" y="0" drill="1.1938" shape="octagon"/>
<text x="-18.3642" y="9.3472" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-10.8458" y="-2.8702" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-192X418" urn="urn:adsk.eagle:footprint:23179/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 19.2 x 41.8 mm</description>
<wire x1="-20.32" y1="8.509" x2="20.32" y2="8.509" width="0.1524" layer="21"/>
<wire x1="20.828" y1="8.001" x2="20.828" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-8.509" x2="-20.32" y2="-8.509" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-8.001" x2="-20.828" y2="8.001" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="8.509" x2="20.828" y2="8.001" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-8.509" x2="20.828" y2="-8.001" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-8.001" x2="-20.32" y2="-8.509" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="8.001" x2="-20.32" y2="8.509" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.447" y="8.89" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-203X418" urn="urn:adsk.eagle:footprint:23180/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 20.3 x 41.8 mm</description>
<wire x1="-20.32" y1="10.16" x2="20.32" y2="10.16" width="0.1524" layer="21"/>
<wire x1="20.828" y1="9.652" x2="20.828" y2="-9.652" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-10.16" x2="-20.32" y2="-10.16" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-9.652" x2="-20.828" y2="9.652" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="10.16" x2="20.828" y2="9.652" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-10.16" x2="20.828" y2="-9.652" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-9.652" x2="-20.32" y2="-10.16" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="9.652" x2="-20.32" y2="10.16" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.32" y="10.541" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C050-035X075" urn="urn:adsk.eagle:footprint:23181/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 5 mm, outline 3.5 x 7.5 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.524" x2="-3.683" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.778" x2="3.429" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.524" x2="3.683" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.778" x2="-3.429" y2="1.778" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.778" x2="3.683" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.778" x2="3.683" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.524" x2="-3.429" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.524" x2="-3.429" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<text x="-3.556" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.556" y="-3.429" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C375-155X418" urn="urn:adsk.eagle:footprint:23182/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 37.5 mm, outline 15.5 x 41.8 mm</description>
<wire x1="-20.32" y1="7.62" x2="20.32" y2="7.62" width="0.1524" layer="21"/>
<wire x1="20.828" y1="7.112" x2="20.828" y2="-7.112" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-7.62" x2="-20.32" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-7.112" x2="-20.828" y2="7.112" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="7.62" x2="20.828" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-7.62" x2="20.828" y2="-7.112" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-7.112" x2="-20.32" y2="-7.62" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="7.112" x2="-20.32" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" shape="octagon"/>
<text x="-20.447" y="8.001" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C075-063X106" urn="urn:adsk.eagle:footprint:23183/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 7.5 mm, outline 6.3 x 10.6 mm</description>
<wire x1="4.953" y1="3.048" x2="-4.953" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.794" x2="-5.207" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-3.048" x2="4.953" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.794" x2="5.207" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="3.048" x2="5.207" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.953" y1="-3.048" x2="5.207" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="-2.794" x2="-4.953" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="2.794" x2="-4.953" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" shape="octagon"/>
<text x="-4.826" y="3.429" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-154X316" urn="urn:adsk.eagle:footprint:23184/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 15.4 x 31.6 mm</description>
<wire x1="-15.24" y1="7.62" x2="15.24" y2="7.62" width="0.1524" layer="21"/>
<wire x1="15.748" y1="7.112" x2="15.748" y2="-7.112" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-7.62" x2="-15.24" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-7.112" x2="-15.748" y2="7.112" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="7.62" x2="15.748" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-7.62" x2="15.748" y2="-7.112" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-7.112" x2="-15.24" y2="-7.62" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="7.112" x2="-15.24" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="8.001" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C275-173X316" urn="urn:adsk.eagle:footprint:23185/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
grid 27.5 mm, outline 17.3 x 31.6 mm</description>
<wire x1="-15.24" y1="8.509" x2="15.24" y2="8.509" width="0.1524" layer="21"/>
<wire x1="15.748" y1="8.001" x2="15.748" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-8.509" x2="-15.24" y2="-8.509" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-8.001" x2="-15.748" y2="8.001" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="8.509" x2="15.748" y2="8.001" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-8.509" x2="15.748" y2="-8.001" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-8.001" x2="-15.24" y2="-8.509" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="8.001" x2="-15.24" y2="8.509" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" shape="octagon"/>
<text x="-15.24" y="8.89" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C0402K" urn="urn:adsk.eagle:footprint:23186/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0204 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 1005</description>
<wire x1="-0.425" y1="0.2" x2="0.425" y2="0.2" width="0.1016" layer="51"/>
<wire x1="0.425" y1="-0.2" x2="-0.425" y2="-0.2" width="0.1016" layer="51"/>
<smd name="1" x="-0.6" y="0" dx="0.925" dy="0.74" layer="1"/>
<smd name="2" x="0.6" y="0" dx="0.925" dy="0.74" layer="1"/>
<text x="-0.5" y="0.425" size="1.016" layer="25">&gt;NAME</text>
<text x="-0.5" y="-1.45" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-0.5" y1="-0.25" x2="-0.225" y2="0.25" layer="51"/>
<rectangle x1="0.225" y1="-0.25" x2="0.5" y2="0.25" layer="51"/>
</package>
<package name="C0603K" urn="urn:adsk.eagle:footprint:23187/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0603 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 1608</description>
<wire x1="-0.725" y1="0.35" x2="0.725" y2="0.35" width="0.1016" layer="51"/>
<wire x1="0.725" y1="-0.35" x2="-0.725" y2="-0.35" width="0.1016" layer="51"/>
<smd name="1" x="-0.875" y="0" dx="1.05" dy="1.08" layer="1"/>
<smd name="2" x="0.875" y="0" dx="1.05" dy="1.08" layer="1"/>
<text x="-0.8" y="0.65" size="1.016" layer="25">&gt;NAME</text>
<text x="-0.8" y="-1.65" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-0.8" y1="-0.4" x2="-0.45" y2="0.4" layer="51"/>
<rectangle x1="0.45" y1="-0.4" x2="0.8" y2="0.4" layer="51"/>
</package>
<package name="C0805K" urn="urn:adsk.eagle:footprint:23188/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 0805 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 2012</description>
<wire x1="-0.925" y1="0.6" x2="0.925" y2="0.6" width="0.1016" layer="51"/>
<wire x1="0.925" y1="-0.6" x2="-0.925" y2="-0.6" width="0.1016" layer="51"/>
<smd name="1" x="-1" y="0" dx="1.3" dy="1.6" layer="1"/>
<smd name="2" x="1" y="0" dx="1.3" dy="1.6" layer="1"/>
<text x="-1" y="0.875" size="1.016" layer="25">&gt;NAME</text>
<text x="-1" y="-1.9" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1" y1="-0.65" x2="-0.5" y2="0.65" layer="51"/>
<rectangle x1="0.5" y1="-0.65" x2="1" y2="0.65" layer="51"/>
</package>
<package name="C1206K" urn="urn:adsk.eagle:footprint:23189/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1206 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 3216</description>
<wire x1="-1.525" y1="0.75" x2="1.525" y2="0.75" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-0.75" x2="-1.525" y2="-0.75" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.5" dy="2" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.5" dy="2" layer="1"/>
<text x="-1.6" y="1.1" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-2.1" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-0.8" x2="-1.1" y2="0.8" layer="51"/>
<rectangle x1="1.1" y1="-0.8" x2="1.6" y2="0.8" layer="51"/>
</package>
<package name="C1210K" urn="urn:adsk.eagle:footprint:23190/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1210 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 3225</description>
<wire x1="-1.525" y1="1.175" x2="1.525" y2="1.175" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-1.175" x2="-1.525" y2="-1.175" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.5" dy="2.9" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.5" dy="2.9" layer="1"/>
<text x="-1.6" y="1.55" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-2.575" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-1.25" x2="-1.1" y2="1.25" layer="51"/>
<rectangle x1="1.1" y1="-1.25" x2="1.6" y2="1.25" layer="51"/>
</package>
<package name="C1812K" urn="urn:adsk.eagle:footprint:23191/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1812 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 4532</description>
<wire x1="-2.175" y1="1.525" x2="2.175" y2="1.525" width="0.1016" layer="51"/>
<wire x1="2.175" y1="-1.525" x2="-2.175" y2="-1.525" width="0.1016" layer="51"/>
<smd name="1" x="-2.05" y="0" dx="1.8" dy="3.7" layer="1"/>
<smd name="2" x="2.05" y="0" dx="1.8" dy="3.7" layer="1"/>
<text x="-2.25" y="1.95" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.25" y="-2.975" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.25" y1="-1.6" x2="-1.65" y2="1.6" layer="51"/>
<rectangle x1="1.65" y1="-1.6" x2="2.25" y2="1.6" layer="51"/>
</package>
<package name="C1825K" urn="urn:adsk.eagle:footprint:23192/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 1825 reflow solder&lt;/b&gt;&lt;p&gt;
Metric Code Size 4564</description>
<wire x1="-1.525" y1="3.125" x2="1.525" y2="3.125" width="0.1016" layer="51"/>
<wire x1="1.525" y1="-3.125" x2="-1.525" y2="-3.125" width="0.1016" layer="51"/>
<smd name="1" x="-1.5" y="0" dx="1.8" dy="6.9" layer="1"/>
<smd name="2" x="1.5" y="0" dx="1.8" dy="6.9" layer="1"/>
<text x="-1.6" y="3.55" size="1.016" layer="25">&gt;NAME</text>
<text x="-1.6" y="-4.625" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="-3.2" x2="-1.1" y2="3.2" layer="51"/>
<rectangle x1="1.1" y1="-3.2" x2="1.6" y2="3.2" layer="51"/>
</package>
<package name="C2220K" urn="urn:adsk.eagle:footprint:23193/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 2220 reflow solder&lt;/b&gt;&lt;p&gt;Metric Code Size 5650</description>
<wire x1="-2.725" y1="2.425" x2="2.725" y2="2.425" width="0.1016" layer="51"/>
<wire x1="2.725" y1="-2.425" x2="-2.725" y2="-2.425" width="0.1016" layer="51"/>
<smd name="1" x="-2.55" y="0" dx="1.85" dy="5.5" layer="1"/>
<smd name="2" x="2.55" y="0" dx="1.85" dy="5.5" layer="1"/>
<text x="-2.8" y="2.95" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.8" y="-3.975" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.8" y1="-2.5" x2="-2.2" y2="2.5" layer="51"/>
<rectangle x1="2.2" y1="-2.5" x2="2.8" y2="2.5" layer="51"/>
</package>
<package name="C2225K" urn="urn:adsk.eagle:footprint:23194/1" library_version="11">
<description>&lt;b&gt;Ceramic Chip Capacitor KEMET 2225 reflow solder&lt;/b&gt;&lt;p&gt;Metric Code Size 5664</description>
<wire x1="-2.725" y1="3.075" x2="2.725" y2="3.075" width="0.1016" layer="51"/>
<wire x1="2.725" y1="-3.075" x2="-2.725" y2="-3.075" width="0.1016" layer="51"/>
<smd name="1" x="-2.55" y="0" dx="1.85" dy="6.8" layer="1"/>
<smd name="2" x="2.55" y="0" dx="1.85" dy="6.8" layer="1"/>
<text x="-2.8" y="3.6" size="1.016" layer="25">&gt;NAME</text>
<text x="-2.8" y="-4.575" size="1.016" layer="27">&gt;VALUE</text>
<rectangle x1="-2.8" y1="-3.15" x2="-2.2" y2="3.15" layer="51"/>
<rectangle x1="2.2" y1="-3.15" x2="2.8" y2="3.15" layer="51"/>
</package>
<package name="C0201" urn="urn:adsk.eagle:footprint:23196/1" library_version="11">
<description>Source: http://www.avxcorp.com/docs/catalogs/cx5r.pdf</description>
<smd name="1" x="-0.25" y="0" dx="0.25" dy="0.35" layer="1"/>
<smd name="2" x="0.25" y="0" dx="0.25" dy="0.35" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.3" y1="-0.15" x2="-0.15" y2="0.15" layer="51"/>
<rectangle x1="0.15" y1="-0.15" x2="0.3" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="0.1" x2="0.15" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="-0.15" x2="0.15" y2="-0.1" layer="51"/>
</package>
<package name="C1808" urn="urn:adsk.eagle:footprint:23197/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Source: AVX .. aphvc.pdf</description>
<wire x1="-1.4732" y1="0.9502" x2="1.4732" y2="0.9502" width="0.1016" layer="51"/>
<wire x1="-1.4478" y1="-0.9502" x2="1.4732" y2="-0.9502" width="0.1016" layer="51"/>
<smd name="1" x="-1.95" y="0" dx="1.6" dy="2.2" layer="1"/>
<smd name="2" x="1.95" y="0" dx="1.6" dy="2.2" layer="1"/>
<text x="-2.233" y="1.827" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.233" y="-2.842" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-2.275" y1="-1.015" x2="-1.225" y2="1.015" layer="51"/>
<rectangle x1="1.225" y1="-1.015" x2="2.275" y2="1.015" layer="51"/>
</package>
<package name="C3640" urn="urn:adsk.eagle:footprint:23198/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;&lt;p&gt;
Source: AVX .. aphvc.pdf</description>
<wire x1="-3.8322" y1="5.0496" x2="3.8322" y2="5.0496" width="0.1016" layer="51"/>
<wire x1="-3.8322" y1="-5.0496" x2="3.8322" y2="-5.0496" width="0.1016" layer="51"/>
<smd name="1" x="-4.267" y="0" dx="2.6" dy="10.7" layer="1"/>
<smd name="2" x="4.267" y="0" dx="2.6" dy="10.7" layer="1"/>
<text x="-4.647" y="6.465" size="1.27" layer="25">&gt;NAME</text>
<text x="-4.647" y="-7.255" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-4.57" y1="-5.1" x2="-3.05" y2="5.1" layer="51"/>
<rectangle x1="3.05" y1="-5.1" x2="4.5688" y2="5.1" layer="51"/>
</package>
<package name="C01005" urn="urn:adsk.eagle:footprint:23199/1" library_version="11">
<description>&lt;b&gt;CAPACITOR&lt;/b&gt;</description>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
<rectangle x1="-0.2" y1="-0.1" x2="-0.075" y2="0.1" layer="51"/>
<rectangle x1="0.075" y1="-0.1" x2="0.2" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="0.05" x2="0.15" y2="0.1" layer="51"/>
<rectangle x1="-0.15" y1="-0.1" x2="0.15" y2="-0.05" layer="51"/>
<smd name="1" x="-0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<smd name="2" x="0.1625" y="0" dx="0.2" dy="0.25" layer="1"/>
<text x="-0.4" y="0.3" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.4" y="-1.6" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="R0402" urn="urn:adsk.eagle:package:23547/3" type="model" library_version="11">
<description>Chip RESISTOR 0402 EIA (1005 Metric)</description>
<packageinstances>
<packageinstance name="R0402"/>
</packageinstances>
</package3d>
<package3d name="R0603" urn="urn:adsk.eagle:package:23555/3" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0603"/>
</packageinstances>
</package3d>
<package3d name="R0805" urn="urn:adsk.eagle:package:23553/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0805"/>
</packageinstances>
</package3d>
<package3d name="R0805W" urn="urn:adsk.eagle:package:23537/2" type="model" library_version="11">
<description>RESISTOR wave soldering</description>
<packageinstances>
<packageinstance name="R0805W"/>
</packageinstances>
</package3d>
<package3d name="R1206" urn="urn:adsk.eagle:package:23540/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1206"/>
</packageinstances>
</package3d>
<package3d name="R1206W" urn="urn:adsk.eagle:package:23539/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R1206W"/>
</packageinstances>
</package3d>
<package3d name="R1210" urn="urn:adsk.eagle:package:23554/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1210"/>
</packageinstances>
</package3d>
<package3d name="R1210W" urn="urn:adsk.eagle:package:23541/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R1210W"/>
</packageinstances>
</package3d>
<package3d name="R2010" urn="urn:adsk.eagle:package:23551/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2010"/>
</packageinstances>
</package3d>
<package3d name="R2010W" urn="urn:adsk.eagle:package:23542/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2010W"/>
</packageinstances>
</package3d>
<package3d name="R2012" urn="urn:adsk.eagle:package:23543/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2012"/>
</packageinstances>
</package3d>
<package3d name="R2012W" urn="urn:adsk.eagle:package:23544/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2012W"/>
</packageinstances>
</package3d>
<package3d name="R2512" urn="urn:adsk.eagle:package:23545/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2512"/>
</packageinstances>
</package3d>
<package3d name="R2512W" urn="urn:adsk.eagle:package:23565/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R2512W"/>
</packageinstances>
</package3d>
<package3d name="R3216" urn="urn:adsk.eagle:package:23557/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R3216"/>
</packageinstances>
</package3d>
<package3d name="R3216W" urn="urn:adsk.eagle:package:23548/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R3216W"/>
</packageinstances>
</package3d>
<package3d name="R3225" urn="urn:adsk.eagle:package:23549/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R3225"/>
</packageinstances>
</package3d>
<package3d name="R3225W" urn="urn:adsk.eagle:package:23550/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R3225W"/>
</packageinstances>
</package3d>
<package3d name="R5025" urn="urn:adsk.eagle:package:23552/2" type="model" library_version="11">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R5025"/>
</packageinstances>
</package3d>
<package3d name="R5025W" urn="urn:adsk.eagle:package:23558/2" type="model" library_version="11">
<description>RESISTOR
wave soldering</description>
<packageinstances>
<packageinstance name="R5025W"/>
</packageinstances>
</package3d>
<package3d name="R6332" urn="urn:adsk.eagle:package:23559/2" type="model" library_version="11">
<description>RESISTOR
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<packageinstances>
<packageinstance name="R6332"/>
</packageinstances>
</package3d>
<package3d name="R6332W" urn="urn:adsk.eagle:package:26078/2" type="model" library_version="11">
<description>RESISTOR wave soldering
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<packageinstances>
<packageinstance name="R6332W"/>
</packageinstances>
</package3d>
<package3d name="M0805" urn="urn:adsk.eagle:package:23556/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.10 W</description>
<packageinstances>
<packageinstance name="M0805"/>
</packageinstances>
</package3d>
<package3d name="M1206" urn="urn:adsk.eagle:package:23566/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M1206"/>
</packageinstances>
</package3d>
<package3d name="M1406" urn="urn:adsk.eagle:package:23569/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.12 W</description>
<packageinstances>
<packageinstance name="M1406"/>
</packageinstances>
</package3d>
<package3d name="M2012" urn="urn:adsk.eagle:package:23561/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.10 W</description>
<packageinstances>
<packageinstance name="M2012"/>
</packageinstances>
</package3d>
<package3d name="M2309" urn="urn:adsk.eagle:package:23562/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M2309"/>
</packageinstances>
</package3d>
<package3d name="M3216" urn="urn:adsk.eagle:package:23563/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M3216"/>
</packageinstances>
</package3d>
<package3d name="M3516" urn="urn:adsk.eagle:package:23573/2" type="model" library_version="11">
<description>RESISTOR
MELF 0.12 W</description>
<packageinstances>
<packageinstance name="M3516"/>
</packageinstances>
</package3d>
<package3d name="M5923" urn="urn:adsk.eagle:package:23564/3" type="model" library_version="11">
<description>RESISTOR
MELF 0.25 W</description>
<packageinstances>
<packageinstance name="M5923"/>
</packageinstances>
</package3d>
<package3d name="0204/5" urn="urn:adsk.eagle:package:23488/1" type="box" library_version="11">
<description>RESISTOR
type 0204, grid 5 mm</description>
<packageinstances>
<packageinstance name="0204/5"/>
</packageinstances>
</package3d>
<package3d name="0204/7" urn="urn:adsk.eagle:package:23498/2" type="model" library_version="11">
<description>RESISTOR
type 0204, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0204/7"/>
</packageinstances>
</package3d>
<package3d name="0207/10" urn="urn:adsk.eagle:package:23491/2" type="model" library_version="11">
<description>RESISTOR
type 0207, grid 10 mm</description>
<packageinstances>
<packageinstance name="0207/10"/>
</packageinstances>
</package3d>
<package3d name="0207/12" urn="urn:adsk.eagle:package:23489/1" type="box" library_version="11">
<description>RESISTOR
type 0207, grid 12 mm</description>
<packageinstances>
<packageinstance name="0207/12"/>
</packageinstances>
</package3d>
<package3d name="0207/15" urn="urn:adsk.eagle:package:23492/1" type="box" library_version="11">
<description>RESISTOR
type 0207, grid 15mm</description>
<packageinstances>
<packageinstance name="0207/15"/>
</packageinstances>
</package3d>
<package3d name="0207/2V" urn="urn:adsk.eagle:package:23490/1" type="box" library_version="11">
<description>RESISTOR
type 0207, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0207/2V"/>
</packageinstances>
</package3d>
<package3d name="0207/5V" urn="urn:adsk.eagle:package:23502/1" type="box" library_version="11">
<description>RESISTOR
type 0207, grid 5 mm</description>
<packageinstances>
<packageinstance name="0207/5V"/>
</packageinstances>
</package3d>
<package3d name="0207/7" urn="urn:adsk.eagle:package:23493/2" type="model" library_version="11">
<description>RESISTOR
type 0207, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0207/7"/>
</packageinstances>
</package3d>
<package3d name="0309/10" urn="urn:adsk.eagle:package:23567/2" type="model" library_version="11">
<description>RESISTOR
type 0309, grid 10mm</description>
<packageinstances>
<packageinstance name="0309/10"/>
</packageinstances>
</package3d>
<package3d name="0309/12" urn="urn:adsk.eagle:package:23571/1" type="box" library_version="11">
<description>RESISTOR
type 0309, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="0309/12"/>
</packageinstances>
</package3d>
<package3d name="0411/12" urn="urn:adsk.eagle:package:23578/1" type="box" library_version="11">
<description>RESISTOR
type 0411, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="0411/12"/>
</packageinstances>
</package3d>
<package3d name="0411/15" urn="urn:adsk.eagle:package:23568/2" type="model" library_version="11">
<description>RESISTOR
type 0411, grid 15 mm</description>
<packageinstances>
<packageinstance name="0411/15"/>
</packageinstances>
</package3d>
<package3d name="0411V" urn="urn:adsk.eagle:package:23570/1" type="box" library_version="11">
<description>RESISTOR
type 0411, grid 3.81 mm</description>
<packageinstances>
<packageinstance name="0411V"/>
</packageinstances>
</package3d>
<package3d name="0414/15" urn="urn:adsk.eagle:package:23579/2" type="model" library_version="11">
<description>RESISTOR
type 0414, grid 15 mm</description>
<packageinstances>
<packageinstance name="0414/15"/>
</packageinstances>
</package3d>
<package3d name="0414V" urn="urn:adsk.eagle:package:23574/1" type="box" library_version="11">
<description>RESISTOR
type 0414, grid 5 mm</description>
<packageinstances>
<packageinstance name="0414V"/>
</packageinstances>
</package3d>
<package3d name="0617/17" urn="urn:adsk.eagle:package:23575/2" type="model" library_version="11">
<description>RESISTOR
type 0617, grid 17.5 mm</description>
<packageinstances>
<packageinstance name="0617/17"/>
</packageinstances>
</package3d>
<package3d name="0617/22" urn="urn:adsk.eagle:package:23577/1" type="box" library_version="11">
<description>RESISTOR
type 0617, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="0617/22"/>
</packageinstances>
</package3d>
<package3d name="0617V" urn="urn:adsk.eagle:package:23576/1" type="box" library_version="11">
<description>RESISTOR
type 0617, grid 5 mm</description>
<packageinstances>
<packageinstance name="0617V"/>
</packageinstances>
</package3d>
<package3d name="0922/22" urn="urn:adsk.eagle:package:23580/2" type="model" library_version="11">
<description>RESISTOR
type 0922, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="0922/22"/>
</packageinstances>
</package3d>
<package3d name="P0613V" urn="urn:adsk.eagle:package:23582/1" type="box" library_version="11">
<description>RESISTOR
type 0613, grid 5 mm</description>
<packageinstances>
<packageinstance name="P0613V"/>
</packageinstances>
</package3d>
<package3d name="P0613/15" urn="urn:adsk.eagle:package:23581/2" type="model" library_version="11">
<description>RESISTOR
type 0613, grid 15 mm</description>
<packageinstances>
<packageinstance name="P0613/15"/>
</packageinstances>
</package3d>
<package3d name="P0817/22" urn="urn:adsk.eagle:package:23583/1" type="box" library_version="11">
<description>RESISTOR
type 0817, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="P0817/22"/>
</packageinstances>
</package3d>
<package3d name="P0817V" urn="urn:adsk.eagle:package:23608/1" type="box" library_version="11">
<description>RESISTOR
type 0817, grid 6.35 mm</description>
<packageinstances>
<packageinstance name="P0817V"/>
</packageinstances>
</package3d>
<package3d name="V234/12" urn="urn:adsk.eagle:package:23592/1" type="box" library_version="11">
<description>RESISTOR
type V234, grid 12.5 mm</description>
<packageinstances>
<packageinstance name="V234/12"/>
</packageinstances>
</package3d>
<package3d name="V235/17" urn="urn:adsk.eagle:package:23586/2" type="model" library_version="11">
<description>RESISTOR
type V235, grid 17.78 mm</description>
<packageinstances>
<packageinstance name="V235/17"/>
</packageinstances>
</package3d>
<package3d name="V526-0" urn="urn:adsk.eagle:package:23590/1" type="box" library_version="11">
<description>RESISTOR
type V526-0, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="V526-0"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102AX" urn="urn:adsk.eagle:package:23594/1" type="box" library_version="11">
<description>Mini MELF 0102 Axial</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102AX"/>
</packageinstances>
</package3d>
<package3d name="0922V" urn="urn:adsk.eagle:package:23589/1" type="box" library_version="11">
<description>RESISTOR
type 0922, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0922V"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102R" urn="urn:adsk.eagle:package:23591/2" type="model" library_version="11">
<description>CECC Size RC2211 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0102W" urn="urn:adsk.eagle:package:23588/2" type="model" library_version="11">
<description>CECC Size RC2211 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0102W"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0204R" urn="urn:adsk.eagle:package:26109/2" type="model" library_version="11">
<description>CECC Size RC3715 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0204R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0204W" urn="urn:adsk.eagle:package:26111/2" type="model" library_version="11">
<description>CECC Size RC3715 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0204W"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0207R" urn="urn:adsk.eagle:package:26113/2" type="model" library_version="11">
<description>CECC Size RC6123 Reflow Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0207R"/>
</packageinstances>
</package3d>
<package3d name="MINI_MELF-0207W" urn="urn:adsk.eagle:package:26112/2" type="model" library_version="11">
<description>CECC Size RC6123 Wave Soldering
source Beyschlag</description>
<packageinstances>
<packageinstance name="MINI_MELF-0207W"/>
</packageinstances>
</package3d>
<package3d name="RDH/15" urn="urn:adsk.eagle:package:23595/1" type="box" library_version="11">
<description>RESISTOR
type RDH, grid 15 mm</description>
<packageinstances>
<packageinstance name="RDH/15"/>
</packageinstances>
</package3d>
<package3d name="0204V" urn="urn:adsk.eagle:package:23495/1" type="box" library_version="11">
<description>RESISTOR
type 0204, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0204V"/>
</packageinstances>
</package3d>
<package3d name="0309V" urn="urn:adsk.eagle:package:23572/1" type="box" library_version="11">
<description>RESISTOR
type 0309, grid 2.5 mm</description>
<packageinstances>
<packageinstance name="0309V"/>
</packageinstances>
</package3d>
<package3d name="R0201" urn="urn:adsk.eagle:package:26117/2" type="model" library_version="11">
<description>RESISTOR chip
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<packageinstances>
<packageinstance name="R0201"/>
</packageinstances>
</package3d>
<package3d name="VMTA55" urn="urn:adsk.eagle:package:26121/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RNC55
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VMTA55"/>
</packageinstances>
</package3d>
<package3d name="VMTB60" urn="urn:adsk.eagle:package:26122/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RNC60
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VMTB60"/>
</packageinstances>
</package3d>
<package3d name="VTA52" urn="urn:adsk.eagle:package:26116/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR52
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA52"/>
</packageinstances>
</package3d>
<package3d name="VTA53" urn="urn:adsk.eagle:package:26118/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR53
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA53"/>
</packageinstances>
</package3d>
<package3d name="VTA54" urn="urn:adsk.eagle:package:26119/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR54
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA54"/>
</packageinstances>
</package3d>
<package3d name="VTA55" urn="urn:adsk.eagle:package:26120/2" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR55
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA55"/>
</packageinstances>
</package3d>
<package3d name="VTA56" urn="urn:adsk.eagle:package:26129/3" type="model" library_version="11">
<description>Bulk Metal® Foil Technology, Tubular Axial Lead Resistors, Meets or Exceeds MIL-R-39005 Requirements
MIL SIZE RBR56
Source: VISHAY .. vta56.pdf</description>
<packageinstances>
<packageinstance name="VTA56"/>
</packageinstances>
</package3d>
<package3d name="R4527" urn="urn:adsk.eagle:package:13310/2" type="model" library_version="11">
<description>Package 4527
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<packageinstances>
<packageinstance name="R4527"/>
</packageinstances>
</package3d>
<package3d name="WSC0001" urn="urn:adsk.eagle:package:26123/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC0001"/>
</packageinstances>
</package3d>
<package3d name="WSC0002" urn="urn:adsk.eagle:package:26125/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC0002"/>
</packageinstances>
</package3d>
<package3d name="WSC01/2" urn="urn:adsk.eagle:package:26127/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC01/2"/>
</packageinstances>
</package3d>
<package3d name="WSC2515" urn="urn:adsk.eagle:package:26134/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC2515"/>
</packageinstances>
</package3d>
<package3d name="WSC4527" urn="urn:adsk.eagle:package:26126/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC4527"/>
</packageinstances>
</package3d>
<package3d name="WSC6927" urn="urn:adsk.eagle:package:26128/2" type="model" library_version="11">
<description>Wirewound Resistors, Precision Power
Source: VISHAY wscwsn.pdf</description>
<packageinstances>
<packageinstance name="WSC6927"/>
</packageinstances>
</package3d>
<package3d name="R1218" urn="urn:adsk.eagle:package:26131/2" type="model" library_version="11">
<description>CRCW1218 Thick Film, Rectangular Chip Resistors
Source: http://www.vishay.com .. dcrcw.pdf</description>
<packageinstances>
<packageinstance name="R1218"/>
</packageinstances>
</package3d>
<package3d name="1812X7R" urn="urn:adsk.eagle:package:26130/2" type="model" library_version="11">
<description>Chip Monolithic Ceramic Capacitors Medium Voltage High Capacitance for General Use
Source: http://www.murata.com .. GRM43DR72E224KW01.pdf</description>
<packageinstances>
<packageinstance name="1812X7R"/>
</packageinstances>
</package3d>
<package3d name="R01005" urn="urn:adsk.eagle:package:26133/2" type="model" library_version="11">
<description>Chip, 0.40 X 0.20 X 0.16 mm body
&lt;p&gt;Chip package with body size 0.40 X 0.20 X 0.16 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="R01005"/>
</packageinstances>
</package3d>
<package3d name="CAPC1005X60" urn="urn:adsk.eagle:package:23626/2" type="model" library_version="11">
<description>Chip, 1.00 X 0.50 X 0.60 mm body
&lt;p&gt;Chip package with body size 1.00 X 0.50 X 0.60 mm&lt;/p&gt;</description>
<packageinstances>
<packageinstance name="C0402"/>
</packageinstances>
</package3d>
<package3d name="C0504" urn="urn:adsk.eagle:package:23624/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0504"/>
</packageinstances>
</package3d>
<package3d name="C0603" urn="urn:adsk.eagle:package:23616/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0603"/>
</packageinstances>
</package3d>
<package3d name="C0805" urn="urn:adsk.eagle:package:23617/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C0805"/>
</packageinstances>
</package3d>
<package3d name="C1206" urn="urn:adsk.eagle:package:23618/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1206"/>
</packageinstances>
</package3d>
<package3d name="C1210" urn="urn:adsk.eagle:package:23619/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1210"/>
</packageinstances>
</package3d>
<package3d name="C1310" urn="urn:adsk.eagle:package:23620/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1310"/>
</packageinstances>
</package3d>
<package3d name="C1608" urn="urn:adsk.eagle:package:23621/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1608"/>
</packageinstances>
</package3d>
<package3d name="C1812" urn="urn:adsk.eagle:package:23622/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1812"/>
</packageinstances>
</package3d>
<package3d name="C1825" urn="urn:adsk.eagle:package:23623/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C1825"/>
</packageinstances>
</package3d>
<package3d name="C2012" urn="urn:adsk.eagle:package:23625/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C2012"/>
</packageinstances>
</package3d>
<package3d name="C3216" urn="urn:adsk.eagle:package:23628/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C3216"/>
</packageinstances>
</package3d>
<package3d name="C3225" urn="urn:adsk.eagle:package:23655/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C3225"/>
</packageinstances>
</package3d>
<package3d name="C4532" urn="urn:adsk.eagle:package:23627/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C4532"/>
</packageinstances>
</package3d>
<package3d name="C4564" urn="urn:adsk.eagle:package:23648/2" type="model" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C4564"/>
</packageinstances>
</package3d>
<package3d name="C025-024X044" urn="urn:adsk.eagle:package:23630/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 mm, outline 2.4 x 4.4 mm</description>
<packageinstances>
<packageinstance name="C025-024X044"/>
</packageinstances>
</package3d>
<package3d name="C025-025X050" urn="urn:adsk.eagle:package:23629/2" type="model" library_version="11">
<description>CAPACITOR
grid 2.5 mm, outline 2.5 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-025X050"/>
</packageinstances>
</package3d>
<package3d name="C025-030X050" urn="urn:adsk.eagle:package:23631/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 mm, outline 3 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-030X050"/>
</packageinstances>
</package3d>
<package3d name="C025-040X050" urn="urn:adsk.eagle:package:23634/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 mm, outline 4 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-040X050"/>
</packageinstances>
</package3d>
<package3d name="C025-050X050" urn="urn:adsk.eagle:package:23633/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 mm, outline 5 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-050X050"/>
</packageinstances>
</package3d>
<package3d name="C025-060X050" urn="urn:adsk.eagle:package:23632/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 mm, outline 6 x 5 mm</description>
<packageinstances>
<packageinstance name="C025-060X050"/>
</packageinstances>
</package3d>
<package3d name="C025_050-024X070" urn="urn:adsk.eagle:package:23639/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 mm + 5 mm, outline 2.4 x 7 mm</description>
<packageinstances>
<packageinstance name="C025_050-024X070"/>
</packageinstances>
</package3d>
<package3d name="C025_050-025X075" urn="urn:adsk.eagle:package:23641/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 2.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-025X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-035X075" urn="urn:adsk.eagle:package:23651/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 3.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-035X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-045X075" urn="urn:adsk.eagle:package:23635/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 4.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-045X075"/>
</packageinstances>
</package3d>
<package3d name="C025_050-055X075" urn="urn:adsk.eagle:package:23636/1" type="box" library_version="11">
<description>CAPACITOR
grid 2.5 + 5 mm, outline 5.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C025_050-055X075"/>
</packageinstances>
</package3d>
<package3d name="C050-024X044" urn="urn:adsk.eagle:package:23643/1" type="box" library_version="11">
<description>CAPACITOR
grid 5 mm, outline 2.4 x 4.4 mm</description>
<packageinstances>
<packageinstance name="C050-024X044"/>
</packageinstances>
</package3d>
<package3d name="C050-025X075" urn="urn:adsk.eagle:package:23637/1" type="box" library_version="11">
<description>CAPACITOR
grid 5 mm, outline 2.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-025X075"/>
</packageinstances>
</package3d>
<package3d name="C050-045X075" urn="urn:adsk.eagle:package:23638/1" type="box" library_version="11">
<description>CAPACITOR
grid 5 mm, outline 4.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-045X075"/>
</packageinstances>
</package3d>
<package3d name="C050-030X075" urn="urn:adsk.eagle:package:23640/1" type="box" library_version="11">
<description>CAPACITOR
grid 5 mm, outline 3 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-030X075"/>
</packageinstances>
</package3d>
<package3d name="C050-050X075" urn="urn:adsk.eagle:package:23665/1" type="box" library_version="11">
<description>CAPACITOR
grid 5 mm, outline 5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-050X075"/>
</packageinstances>
</package3d>
<package3d name="C050-055X075" urn="urn:adsk.eagle:package:23642/1" type="box" library_version="11">
<description>CAPACITOR
grid 5 mm, outline 5.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-055X075"/>
</packageinstances>
</package3d>
<package3d name="C050-075X075" urn="urn:adsk.eagle:package:23645/1" type="box" library_version="11">
<description>CAPACITOR
grid 5 mm, outline 7.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-075X075"/>
</packageinstances>
</package3d>
<package3d name="C050H075X075" urn="urn:adsk.eagle:package:23644/1" type="box" library_version="11">
<description>CAPACITOR
Horizontal, grid 5 mm, outline 7.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050H075X075"/>
</packageinstances>
</package3d>
<package3d name="C075-032X103" urn="urn:adsk.eagle:package:23646/1" type="box" library_version="11">
<description>CAPACITOR
grid 7.5 mm, outline 3.2 x 10.3 mm</description>
<packageinstances>
<packageinstance name="C075-032X103"/>
</packageinstances>
</package3d>
<package3d name="C075-042X103" urn="urn:adsk.eagle:package:23656/1" type="box" library_version="11">
<description>CAPACITOR
grid 7.5 mm, outline 4.2 x 10.3 mm</description>
<packageinstances>
<packageinstance name="C075-042X103"/>
</packageinstances>
</package3d>
<package3d name="C075-052X106" urn="urn:adsk.eagle:package:23650/1" type="box" library_version="11">
<description>CAPACITOR
grid 7.5 mm, outline 5.2 x 10.6 mm</description>
<packageinstances>
<packageinstance name="C075-052X106"/>
</packageinstances>
</package3d>
<package3d name="C102-043X133" urn="urn:adsk.eagle:package:23647/1" type="box" library_version="11">
<description>CAPACITOR
grid 10.2 mm, outline 4.3 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-043X133"/>
</packageinstances>
</package3d>
<package3d name="C102-054X133" urn="urn:adsk.eagle:package:23649/1" type="box" library_version="11">
<description>CAPACITOR
grid 10.2 mm, outline 5.4 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-054X133"/>
</packageinstances>
</package3d>
<package3d name="C102-064X133" urn="urn:adsk.eagle:package:23653/1" type="box" library_version="11">
<description>CAPACITOR
grid 10.2 mm, outline 6.4 x 13.3 mm</description>
<packageinstances>
<packageinstance name="C102-064X133"/>
</packageinstances>
</package3d>
<package3d name="C102_152-062X184" urn="urn:adsk.eagle:package:23652/1" type="box" library_version="11">
<description>CAPACITOR
grid 10.2 mm + 15.2 mm, outline 6.2 x 18.4 mm</description>
<packageinstances>
<packageinstance name="C102_152-062X184"/>
</packageinstances>
</package3d>
<package3d name="C150-054X183" urn="urn:adsk.eagle:package:23669/1" type="box" library_version="11">
<description>CAPACITOR
grid 15 mm, outline 5.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-054X183"/>
</packageinstances>
</package3d>
<package3d name="C150-064X183" urn="urn:adsk.eagle:package:23654/1" type="box" library_version="11">
<description>CAPACITOR
grid 15 mm, outline 6.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-064X183"/>
</packageinstances>
</package3d>
<package3d name="C150-072X183" urn="urn:adsk.eagle:package:23657/1" type="box" library_version="11">
<description>CAPACITOR
grid 15 mm, outline 7.2 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-072X183"/>
</packageinstances>
</package3d>
<package3d name="C150-084X183" urn="urn:adsk.eagle:package:23658/1" type="box" library_version="11">
<description>CAPACITOR
grid 15 mm, outline 8.4 x 18.3 mm</description>
<packageinstances>
<packageinstance name="C150-084X183"/>
</packageinstances>
</package3d>
<package3d name="C150-091X182" urn="urn:adsk.eagle:package:23659/1" type="box" library_version="11">
<description>CAPACITOR
grid 15 mm, outline 9.1 x 18.2 mm</description>
<packageinstances>
<packageinstance name="C150-091X182"/>
</packageinstances>
</package3d>
<package3d name="C225-062X268" urn="urn:adsk.eagle:package:23661/1" type="box" library_version="11">
<description>CAPACITOR
grid 22.5 mm, outline 6.2 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-062X268"/>
</packageinstances>
</package3d>
<package3d name="C225-074X268" urn="urn:adsk.eagle:package:23660/1" type="box" library_version="11">
<description>CAPACITOR
grid 22.5 mm, outline 7.4 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-074X268"/>
</packageinstances>
</package3d>
<package3d name="C225-087X268" urn="urn:adsk.eagle:package:23662/1" type="box" library_version="11">
<description>CAPACITOR
grid 22.5 mm, outline 8.7 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-087X268"/>
</packageinstances>
</package3d>
<package3d name="C225-108X268" urn="urn:adsk.eagle:package:23663/1" type="box" library_version="11">
<description>CAPACITOR
grid 22.5 mm, outline 10.8 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-108X268"/>
</packageinstances>
</package3d>
<package3d name="C225-113X268" urn="urn:adsk.eagle:package:23667/1" type="box" library_version="11">
<description>CAPACITOR
grid 22.5 mm, outline 11.3 x 26.8 mm</description>
<packageinstances>
<packageinstance name="C225-113X268"/>
</packageinstances>
</package3d>
<package3d name="C275-093X316" urn="urn:adsk.eagle:package:23701/1" type="box" library_version="11">
<description>CAPACITOR
grid 27.5 mm, outline 9.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-093X316"/>
</packageinstances>
</package3d>
<package3d name="C275-113X316" urn="urn:adsk.eagle:package:23673/1" type="box" library_version="11">
<description>CAPACITOR
grid 27.5 mm, outline 11.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-113X316"/>
</packageinstances>
</package3d>
<package3d name="C275-134X316" urn="urn:adsk.eagle:package:23664/1" type="box" library_version="11">
<description>CAPACITOR
grid 27.5 mm, outline 13.4 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-134X316"/>
</packageinstances>
</package3d>
<package3d name="C275-205X316" urn="urn:adsk.eagle:package:23666/1" type="box" library_version="11">
<description>CAPACITOR
grid 27.5 mm, outline 20.5 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-205X316"/>
</packageinstances>
</package3d>
<package3d name="C325-137X374" urn="urn:adsk.eagle:package:23672/1" type="box" library_version="11">
<description>CAPACITOR
grid 32.5 mm, outline 13.7 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-137X374"/>
</packageinstances>
</package3d>
<package3d name="C325-162X374" urn="urn:adsk.eagle:package:23670/1" type="box" library_version="11">
<description>CAPACITOR
grid 32.5 mm, outline 16.2 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-162X374"/>
</packageinstances>
</package3d>
<package3d name="C325-182X374" urn="urn:adsk.eagle:package:23668/1" type="box" library_version="11">
<description>CAPACITOR
grid 32.5 mm, outline 18.2 x 37.4 mm</description>
<packageinstances>
<packageinstance name="C325-182X374"/>
</packageinstances>
</package3d>
<package3d name="C375-192X418" urn="urn:adsk.eagle:package:23674/1" type="box" library_version="11">
<description>CAPACITOR
grid 37.5 mm, outline 19.2 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-192X418"/>
</packageinstances>
</package3d>
<package3d name="C375-203X418" urn="urn:adsk.eagle:package:23671/1" type="box" library_version="11">
<description>CAPACITOR
grid 37.5 mm, outline 20.3 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-203X418"/>
</packageinstances>
</package3d>
<package3d name="C050-035X075" urn="urn:adsk.eagle:package:23677/1" type="box" library_version="11">
<description>CAPACITOR
grid 5 mm, outline 3.5 x 7.5 mm</description>
<packageinstances>
<packageinstance name="C050-035X075"/>
</packageinstances>
</package3d>
<package3d name="C375-155X418" urn="urn:adsk.eagle:package:23675/1" type="box" library_version="11">
<description>CAPACITOR
grid 37.5 mm, outline 15.5 x 41.8 mm</description>
<packageinstances>
<packageinstance name="C375-155X418"/>
</packageinstances>
</package3d>
<package3d name="C075-063X106" urn="urn:adsk.eagle:package:23678/1" type="box" library_version="11">
<description>CAPACITOR
grid 7.5 mm, outline 6.3 x 10.6 mm</description>
<packageinstances>
<packageinstance name="C075-063X106"/>
</packageinstances>
</package3d>
<package3d name="C275-154X316" urn="urn:adsk.eagle:package:23685/1" type="box" library_version="11">
<description>CAPACITOR
grid 27.5 mm, outline 15.4 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-154X316"/>
</packageinstances>
</package3d>
<package3d name="C275-173X316" urn="urn:adsk.eagle:package:23676/1" type="box" library_version="11">
<description>CAPACITOR
grid 27.5 mm, outline 17.3 x 31.6 mm</description>
<packageinstances>
<packageinstance name="C275-173X316"/>
</packageinstances>
</package3d>
<package3d name="C0402K" urn="urn:adsk.eagle:package:23679/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 0204 reflow solder
Metric Code Size 1005</description>
<packageinstances>
<packageinstance name="C0402K"/>
</packageinstances>
</package3d>
<package3d name="C0603K" urn="urn:adsk.eagle:package:23680/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 0603 reflow solder
Metric Code Size 1608</description>
<packageinstances>
<packageinstance name="C0603K"/>
</packageinstances>
</package3d>
<package3d name="C0805K" urn="urn:adsk.eagle:package:23681/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 0805 reflow solder
Metric Code Size 2012</description>
<packageinstances>
<packageinstance name="C0805K"/>
</packageinstances>
</package3d>
<package3d name="C1206K" urn="urn:adsk.eagle:package:23682/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 1206 reflow solder
Metric Code Size 3216</description>
<packageinstances>
<packageinstance name="C1206K"/>
</packageinstances>
</package3d>
<package3d name="C1210K" urn="urn:adsk.eagle:package:23683/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 1210 reflow solder
Metric Code Size 3225</description>
<packageinstances>
<packageinstance name="C1210K"/>
</packageinstances>
</package3d>
<package3d name="C1812K" urn="urn:adsk.eagle:package:23686/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 1812 reflow solder
Metric Code Size 4532</description>
<packageinstances>
<packageinstance name="C1812K"/>
</packageinstances>
</package3d>
<package3d name="C1825K" urn="urn:adsk.eagle:package:23684/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 1825 reflow solder
Metric Code Size 4564</description>
<packageinstances>
<packageinstance name="C1825K"/>
</packageinstances>
</package3d>
<package3d name="C2220K" urn="urn:adsk.eagle:package:23687/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 2220 reflow solderMetric Code Size 5650</description>
<packageinstances>
<packageinstance name="C2220K"/>
</packageinstances>
</package3d>
<package3d name="C2225K" urn="urn:adsk.eagle:package:23692/2" type="model" library_version="11">
<description>Ceramic Chip Capacitor KEMET 2225 reflow solderMetric Code Size 5664</description>
<packageinstances>
<packageinstance name="C2225K"/>
</packageinstances>
</package3d>
<package3d name="C0201" urn="urn:adsk.eagle:package:23690/2" type="model" library_version="11">
<description>Source: http://www.avxcorp.com/docs/catalogs/cx5r.pdf</description>
<packageinstances>
<packageinstance name="C0201"/>
</packageinstances>
</package3d>
<package3d name="C1808" urn="urn:adsk.eagle:package:23689/2" type="model" library_version="11">
<description>CAPACITOR
Source: AVX .. aphvc.pdf</description>
<packageinstances>
<packageinstance name="C1808"/>
</packageinstances>
</package3d>
<package3d name="C3640" urn="urn:adsk.eagle:package:23693/2" type="model" library_version="11">
<description>CAPACITOR
Source: AVX .. aphvc.pdf</description>
<packageinstances>
<packageinstance name="C3640"/>
</packageinstances>
</package3d>
<package3d name="C01005" urn="urn:adsk.eagle:package:23691/1" type="box" library_version="11">
<description>CAPACITOR</description>
<packageinstances>
<packageinstance name="C01005"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="R-US" urn="urn:adsk.eagle:symbol:23200/1" library_version="11">
<wire x1="-2.54" y1="0" x2="-2.159" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-2.159" y1="1.016" x2="-1.524" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-1.524" y1="-1.016" x2="-0.889" y2="1.016" width="0.2032" layer="94"/>
<wire x1="-0.889" y1="1.016" x2="-0.254" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="-0.254" y1="-1.016" x2="0.381" y2="1.016" width="0.2032" layer="94"/>
<wire x1="0.381" y1="1.016" x2="1.016" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="1.016" y1="-1.016" x2="1.651" y2="1.016" width="0.2032" layer="94"/>
<wire x1="1.651" y1="1.016" x2="2.286" y2="-1.016" width="0.2032" layer="94"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="0" width="0.2032" layer="94"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
<symbol name="C-US" urn="urn:adsk.eagle:symbol:23201/1" library_version="11">
<wire x1="-2.54" y1="0" x2="2.54" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="-1.016" x2="0" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="-1" x2="2.4892" y2="-1.8542" width="0.254" layer="94" curve="-37.878202"/>
<wire x1="-2.4668" y1="-1.8504" x2="0" y2="-1.0161" width="0.254" layer="94" curve="-37.373024"/>
<text x="1.016" y="0.635" size="1.778" layer="95">&gt;NAME</text>
<text x="1.016" y="-4.191" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="0" y="2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="R-US_" urn="urn:adsk.eagle:component:23792/22" prefix="R" uservalue="yes" library_version="11">
<description>&lt;B&gt;RESISTOR&lt;/B&gt;, American symbol</description>
<gates>
<gate name="G$1" symbol="R-US" x="0" y="0"/>
</gates>
<devices>
<device name="R0402" package="R0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23547/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="34" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0603" package="R0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23555/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="77" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0805" package="R0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23553/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="85" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0805W" package="R0805W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23537/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1206" package="R1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23540/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="19" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1206W" package="R1206W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23539/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1210" package="R1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23554/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1210W" package="R1210W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23541/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2010" package="R2010">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23551/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2010W" package="R2010W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23542/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2012" package="R2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23543/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2012W" package="R2012W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23544/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2512" package="R2512">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23545/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2512W" package="R2512W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23565/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3216" package="R3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23557/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3216W" package="R3216W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23548/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3225" package="R3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23549/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3225W" package="R3225W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23550/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R5025" package="R5025">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23552/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R5025W" package="R5025W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23558/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R6332" package="R6332">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23559/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R6332W" package="R6332W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26078/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M0805" package="M0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23556/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="45" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M1206" package="M1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23566/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="22" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M1406" package="M1406">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23569/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M2012" package="M2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23561/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M2309" package="M2309">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23562/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M3216" package="M3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23563/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M3516" package="M3516">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23573/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="M5923" package="M5923">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23564/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0204/5" package="0204/5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23488/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="18" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0204/7" package="0204/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23498/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="48" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/10" package="0207/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23491/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="36" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/12" package="0207/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23489/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="3" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/15" package="0207/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23492/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/2V" package="0207/2V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23490/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/5V" package="0207/5V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23502/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/7" package="0207/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23493/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="22" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0309/10" package="0309/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23567/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0309/12" package="0309/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23571/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="8" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0411/12" package="0411/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23578/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0411/15" package="0411/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23568/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0411/3V" package="0411V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23570/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0414/15" package="0414/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23579/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0414/5V" package="0414V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23574/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0617/17" package="0617/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23575/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0617/22" package="0617/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23577/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0617/5V" package="0617V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23576/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0922/22" package="0922/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23580/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0613/5V" package="P0613V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23582/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0613/15" package="P0613/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23581/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0817/22" package="P0817/22">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23583/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0817/7V" package="P0817V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23608/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="V234/12" package="V234/12">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23592/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="V235/17" package="V235/17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23586/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="V526-0" package="V526-0">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23590/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0102AX" package="MINI_MELF-0102AX">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23594/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0922V" package="0922V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23589/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0102R" package="MINI_MELF-0102R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23591/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0102W" package="MINI_MELF-0102W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23588/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0204R" package="MINI_MELF-0204R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26109/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0204W" package="MINI_MELF-0204W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26111/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0207R" package="MINI_MELF-0207R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26113/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="MELF0207W" package="MINI_MELF-0207W">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26112/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="RDH/15" package="RDH/15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23595/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0204/2V" package="0204V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23495/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0309/V" package="0309V">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23572/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0201" package="R0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26117/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VMTA55" package="VMTA55">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26121/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VMTB60" package="VMTB60">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26122/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA52" package="VTA52">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26116/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="3" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA53" package="VTA53">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26118/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA54" package="VTA54">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26119/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA55" package="VTA55">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26120/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="VTA56" package="VTA56">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26129/3"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R4527" package="R4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13310/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC0001" package="WSC0001">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26123/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC0002" package="WSC0002">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26125/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC01/2" package="WSC01/2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26127/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC2515" package="WSC2515">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26134/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC4527" package="WSC4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26126/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="WSC6927" package="WSC6927">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26128/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1218" package="R1218">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26131/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="1812X7R" package="1812X7R">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26130/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="01005" package="R01005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26133/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
</devices>
<spice>
<pinmapping spiceprefix="R">
<pinmap gate="G$1" pin="1" pinorder="1"/>
<pinmap gate="G$1" pin="2" pinorder="2"/>
</pinmapping>
</spice>
</deviceset>
<deviceset name="C-US" urn="urn:adsk.eagle:component:23794/44" prefix="C" uservalue="yes" library_version="11">
<description>&lt;B&gt;CAPACITOR&lt;/B&gt;, American symbol</description>
<gates>
<gate name="G$1" symbol="C-US" x="0" y="0"/>
</gates>
<devices>
<device name="C0402" package="C0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23626/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="16" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0504" package="C0504">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23624/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0603" package="C0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23616/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="37" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0805" package="C0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23617/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="63" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1206" package="C1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23618/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="24" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1210" package="C1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23619/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="6" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1310" package="C1310">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23620/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1608" package="C1608">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23621/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1812" package="C1812">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23622/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1825" package="C1825">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23623/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C2012" package="C2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23625/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C3216" package="C3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23628/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C3225" package="C3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23655/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C4532" package="C4532">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23627/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C4564" package="C4564">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23648/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-024X044" package="C025-024X044">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23630/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="17" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-025X050" package="C025-025X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23629/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-030X050" package="C025-030X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23631/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-040X050" package="C025-040X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23634/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-050X050" package="C025-050X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23633/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="7" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025-060X050" package="C025-060X050">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23632/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C025_050-024X070" package="C025_050-024X070">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23639/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025_050-025X075" package="C025_050-025X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23641/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025_050-035X075" package="C025_050-035X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23651/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025_050-045X075" package="C025_050-045X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23635/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="025_050-055X075" package="C025_050-055X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23636/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-024X044" package="C050-024X044">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23643/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="14" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-025X075" package="C050-025X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23637/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-045X075" package="C050-045X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23638/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-030X075" package="C050-030X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23640/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="5" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-050X075" package="C050-050X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23665/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-055X075" package="C050-055X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23642/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-075X075" package="C050-075X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23645/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050H075X075" package="C050H075X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23644/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="075-032X103" package="C075-032X103">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23646/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="7" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="075-042X103" package="C075-042X103">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23656/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="075-052X106" package="C075-052X106">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23650/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="102-043X133" package="C102-043X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23647/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="102-054X133" package="C102-054X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23649/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="102-064X133" package="C102-064X133">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23653/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="102_152-062X184" package="C102_152-062X184">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23652/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-054X183" package="C150-054X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23669/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-064X183" package="C150-064X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23654/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-072X183" package="C150-072X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23657/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-084X183" package="C150-084X183">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23658/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="150-091X182" package="C150-091X182">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23659/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-062X268" package="C225-062X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23661/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-074X268" package="C225-074X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23660/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-087X268" package="C225-087X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23662/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-108X268" package="C225-108X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23663/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="225-113X268" package="C225-113X268">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23667/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-093X316" package="C275-093X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23701/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-113X316" package="C275-113X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23673/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-134X316" package="C275-134X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23664/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-205X316" package="C275-205X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23666/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="325-137X374" package="C325-137X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23672/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="325-162X374" package="C325-162X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23670/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="325-182X374" package="C325-182X374">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23668/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="375-192X418" package="C375-192X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23674/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="375-203X418" package="C375-203X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23671/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="050-035X075" package="C050-035X075">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23677/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="375-155X418" package="C375-155X418">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23675/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="075-063X106" package="C075-063X106">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23678/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="1" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-154X316" package="C275-154X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23685/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="275-173X316" package="C275-173X316">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23676/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0402K" package="C0402K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23679/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="4" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0603K" package="C0603K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23680/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="5" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0805K" package="C0805K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23681/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="19" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1206K" package="C1206K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23682/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1210K" package="C1210K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23683/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1812K" package="C1812K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23686/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1825K" package="C1825K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23684/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C2220K" package="C2220K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23687/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C2225K" package="C2225K">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23692/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C0201" package="C0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23690/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="3" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C1808" package="C1808">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23689/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="C3640" package="C3640">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23693/2"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="0" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
<device name="01005" package="C01005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:23691/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="2" constant="no"/>
<attribute name="SPICEPREFIX" value="C" constant="no"/>
</technology>
</technologies>
</device>
</devices>
<spice>
<pinmapping spiceprefix="C">
<pinmap gate="G$1" pin="1" pinorder="1"/>
<pinmap gate="G$1" pin="2" pinorder="2"/>
</pinmapping>
</spice>
</deviceset>
</devicesets>
</library>
<library name="TPS61023">
<packages>
<package name="DRL0006A">
<smd name="1" x="-0.74" y="0.5" dx="0.3" dy="0.67" layer="1" rot="R90"/>
<smd name="2" x="-0.74" y="0.000003125" dx="0.3" dy="0.67" layer="1" rot="R90"/>
<smd name="3" x="-0.74" y="-0.5" dx="0.3" dy="0.67" layer="1" rot="R90"/>
<smd name="4" x="0.74" y="-0.5" dx="0.3" dy="0.67" layer="1" rot="R90"/>
<smd name="5" x="0.74" y="0.000003125" dx="0.3" dy="0.67" layer="1" rot="R90"/>
<smd name="6" x="0.74" y="0.5" dx="0.3" dy="0.67" layer="1" rot="R90"/>
<wire x1="-0.635" y1="-0.635" x2="-0.635" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.635" y1="-0.635" x2="0.635" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.6604" y1="-0.8382" x2="-0.6604" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-0.6604" y1="-0.381" x2="-0.6604" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.635" x2="-0.8382" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="0.381" x2="-0.8382" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.6604" y1="-0.8382" x2="0.6604" y2="-0.8382" width="0.1524" layer="51"/>
<wire x1="-0.635" y1="0.381" x2="-0.635" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.635" y1="-0.381" x2="0.6604" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.6604" y1="-0.381" x2="0.8382" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.381" x2="-0.635" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.635" x2="-0.635" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="0.635" y1="-0.635" x2="0.6604" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="0.6604" y1="-0.635" x2="0.8382" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="0.6604" y1="-0.381" x2="0.6604" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-0.635" y1="-0.127" x2="-0.635" y2="0.127" width="0.1524" layer="51"/>
<wire x1="0.635" y1="0.381" x2="0.635" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.635" y1="0.381" x2="0.8382" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-0.6604" y1="0.635" x2="-0.6604" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="0.635" y1="0.635" x2="0.6604" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.6604" y1="0.635" x2="0.8382" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="0.381" x2="0.8382" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.127" x2="-0.635" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.127" x2="-0.8382" y2="0.127" width="0.1524" layer="51"/>
<wire x1="0.6604" y1="0.635" x2="0.6604" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="0.127" x2="-0.6604" y2="0.127" width="0.1524" layer="51"/>
<wire x1="-0.6604" y1="0.127" x2="-0.635" y2="0.127" width="0.1524" layer="51"/>
<wire x1="0.6604" y1="-0.8382" x2="0.6604" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-0.635" x2="0.8382" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.6604" y1="0.8382" x2="0.6604" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="0.635" x2="-0.635" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.6604" y1="0.127" x2="-0.6604" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="0.381" x2="-0.635" y2="0.381" width="0.1524" layer="51"/>
<wire x1="0.635" y1="0.127" x2="0.8382" y2="0.127" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-0.127" x2="0.8382" y2="0.127" width="0.1524" layer="51"/>
<wire x1="0.635" y1="-0.127" x2="0.8382" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="0.635" y1="-0.127" x2="0.635" y2="0.127" width="0.1524" layer="51"/>
<wire x1="-0.1778" y1="0.508" x2="-0.4826" y2="0.508" width="0.1016" layer="51" curve="-180"/>
<wire x1="-0.4826" y1="0.508" x2="-0.1778" y2="0.508" width="0.1016" layer="51" curve="-180"/>
<wire x1="-0.508" y1="-0.9398" x2="0.508" y2="-0.9398" width="0.2032" layer="21"/>
<wire x1="-0.889" y1="0.9398" x2="0.508" y2="0.9398" width="0.2032" layer="21"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
</packages>
<symbols>
<symbol name="TPS61023DRL">
<pin name="FB" x="17.78" y="-5.08" length="middle" direction="in" rot="R180"/>
<pin name="EN" x="-17.78" y="0" length="middle" direction="in"/>
<pin name="VIN" x="-17.78" y="5.08" length="middle" direction="pwr"/>
<pin name="GND" x="-17.78" y="-5.08" length="middle" direction="pwr"/>
<pin name="SW" x="17.78" y="5.08" length="middle" direction="pwr" rot="R180"/>
<pin name="VOUT" x="17.78" y="0" length="middle" direction="pwr" rot="R180"/>
<wire x1="-12.7" y1="-10.16" x2="12.7" y2="-10.16" width="0.2032" layer="94"/>
<wire x1="12.7" y1="-10.16" x2="12.7" y2="10.16" width="0.2032" layer="94"/>
<wire x1="12.7" y1="10.16" x2="-12.7" y2="10.16" width="0.2032" layer="94"/>
<wire x1="-12.7" y1="10.16" x2="-12.7" y2="-10.16" width="0.2032" layer="94"/>
<text x="-4.7244" y="1.4986" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="-5.3594" y="-1.0414" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
<text x="-5.3594" y="-1.0414" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="TPS61023DRLR" prefix="U">
<gates>
<gate name="A" symbol="TPS61023DRL" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DRL0006A">
<connects>
<connect gate="A" pin="EN" pad="2"/>
<connect gate="A" pin="FB" pad="1"/>
<connect gate="A" pin="GND" pad="4"/>
<connect gate="A" pin="SW" pad="5"/>
<connect gate="A" pin="VIN" pad="3"/>
<connect gate="A" pin="VOUT" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="TPS61023DRLR" constant="no"/>
<attribute name="MFR_NAME" value="Texas Instruments" constant="no"/>
<attribute name="REFDES" value="RefDes" constant="no"/>
<attribute name="TYPE" value="TYPE" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="wirepad" urn="urn:adsk.eagle:library:412">
<description>&lt;b&gt;Single Pads&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="1,6/0,8" urn="urn:adsk.eagle:footprint:30809/1" library_version="2">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="-0.762" y1="0.762" x2="-0.508" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="0.762" x2="-0.762" y2="0.508" width="0.1524" layer="21"/>
<wire x1="0.762" y1="0.762" x2="0.762" y2="0.508" width="0.1524" layer="21"/>
<wire x1="0.762" y1="0.762" x2="0.508" y2="0.762" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-0.508" x2="0.762" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="0.762" y1="-0.762" x2="0.508" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-0.508" y1="-0.762" x2="-0.762" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="-0.762" x2="-0.762" y2="-0.508" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.635" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-0.762" y="1.016" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.6" size="0.0254" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="1,6/0,8" urn="urn:adsk.eagle:package:30830/1" type="box" library_version="2">
<description>THROUGH-HOLE PAD</description>
<packageinstances>
<packageinstance name="1,6/0,8"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="PAD" urn="urn:adsk.eagle:symbol:30808/1" library_version="2">
<wire x1="-1.016" y1="1.016" x2="1.016" y2="-1.016" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-1.016" x2="1.016" y2="1.016" width="0.254" layer="94"/>
<text x="-1.143" y="1.8542" size="1.778" layer="95">&gt;NAME</text>
<text x="-1.143" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="P" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="1,6/0,8" urn="urn:adsk.eagle:component:30848/2" prefix="PAD" uservalue="yes" library_version="2">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<gates>
<gate name="P" symbol="PAD" x="0" y="0"/>
</gates>
<devices>
<device name="" package="1,6/0,8">
<connects>
<connect gate="P" pin="P" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:30830/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="15" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="VL53L1X">
<packages>
<package name="LGA12_STM">
<smd name="1" x="0.799971875" y="1.6" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="2" x="0.799971875" y="0.8" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="3" x="0.799971875" y="0" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="4" x="0.799971875" y="-0.8" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="5" x="0.799971875" y="-1.6" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="6" x="0" y="-1.599946875" dx="0.508" dy="0.508" layer="1" rot="R180"/>
<smd name="7" x="-0.799971875" y="-1.6" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="8" x="-0.799971875" y="-0.8" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="9" x="-0.799971875" y="0" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="10" x="-0.799971875" y="0.8" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="11" x="-0.799971875" y="1.6" dx="0.508" dy="0.508" layer="1" rot="R270"/>
<smd name="12" x="0" y="1.599946875" dx="0.508" dy="0.508" layer="1" rot="R180"/>
<wire x1="-0.7874" y1="1.6002" x2="0.7874" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="0.7874" y1="1.6002" x2="2.0066" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="-0.7874" y1="0.7874" x2="2.0066" y2="0.7874" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="0.7874" x2="2.3876" y2="0.7874" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="1.6002" x2="2.0066" y2="2.8702" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="0.7874" x2="2.0066" y2="-0.4826" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="1.6002" x2="1.8796" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="1.6002" x2="2.1336" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="1.8796" y1="1.8542" x2="2.1336" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="0.7874" x2="1.8796" y2="0.5334" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="0.7874" x2="2.1336" y2="0.5334" width="0.1524" layer="47"/>
<wire x1="1.8796" y1="0.5334" x2="2.1336" y2="0.5334" width="0.1524" layer="47"/>
<wire x1="-0.7874" y1="1.6002" x2="-0.7874" y2="6.2992" width="0.1524" layer="47"/>
<wire x1="-0.7874" y1="6.2992" x2="-0.7874" y2="6.6802" width="0.1524" layer="47"/>
<wire x1="0.7874" y1="1.6002" x2="0.7874" y2="6.2992" width="0.1524" layer="47"/>
<wire x1="0.7874" y1="6.2992" x2="0.7874" y2="6.6802" width="0.1524" layer="47"/>
<wire x1="-0.7874" y1="6.2992" x2="-2.0574" y2="6.2992" width="0.1524" layer="47"/>
<wire x1="0.7874" y1="6.2992" x2="2.0574" y2="6.2992" width="0.1524" layer="47"/>
<wire x1="-0.7874" y1="6.2992" x2="-1.0414" y2="6.4262" width="0.1524" layer="47"/>
<wire x1="-0.7874" y1="6.2992" x2="-1.0414" y2="6.1722" width="0.1524" layer="47"/>
<wire x1="-1.0414" y1="6.4262" x2="-1.0414" y2="6.1722" width="0.1524" layer="47"/>
<wire x1="0.7874" y1="6.2992" x2="1.0414" y2="6.4262" width="0.1524" layer="47"/>
<wire x1="0.7874" y1="6.2992" x2="1.0414" y2="6.1722" width="0.1524" layer="47"/>
<wire x1="1.0414" y1="6.4262" x2="1.0414" y2="6.1722" width="0.1524" layer="47"/>
<wire x1="2.0066" y1="1.6002" x2="3.9116" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="3.9116" y1="1.6002" x2="4.2926" y2="1.6002" width="0.1524" layer="47"/>
<wire x1="0" y1="-1.6002" x2="3.9116" y2="-1.6002" width="0.1524" layer="47"/>
<wire x1="3.9116" y1="-1.6002" x2="4.2926" y2="-1.6002" width="0.1524" layer="47"/>
<wire x1="3.9116" y1="1.6002" x2="3.9116" y2="2.8702" width="0.1524" layer="47"/>
<wire x1="3.9116" y1="-1.6002" x2="3.9116" y2="-2.8702" width="0.1524" layer="47"/>
<wire x1="3.9116" y1="1.6002" x2="3.7846" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="3.9116" y1="1.6002" x2="4.0386" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="3.7846" y1="1.8542" x2="4.0386" y2="1.8542" width="0.1524" layer="47"/>
<wire x1="3.9116" y1="-1.6002" x2="3.7846" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="3.9116" y1="-1.6002" x2="4.0386" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="3.7846" y1="-1.8542" x2="4.0386" y2="-1.8542" width="0.1524" layer="47"/>
<wire x1="1.27" y1="2.4892" x2="-1.27" y2="2.4892" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="2.4892" x2="-2.6416" y2="2.4892" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="2.4892" x2="-3.0226" y2="2.4892" width="0.1524" layer="47"/>
<wire x1="1.27" y1="-2.4892" x2="-2.6416" y2="-2.4892" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="-2.4892" x2="-3.0226" y2="-2.4892" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="2.4892" x2="-2.6416" y2="-2.4892" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="2.4892" x2="-2.7686" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="2.4892" x2="-2.5146" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="-2.7686" y1="2.2352" x2="-2.5146" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="-2.4892" x2="-2.7686" y2="-2.2352" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="-2.4892" x2="-2.5146" y2="-2.2352" width="0.1524" layer="47"/>
<wire x1="-2.7686" y1="-2.2352" x2="-2.5146" y2="-2.2352" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="2.4892" x2="-1.27" y2="-5.0292" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="-5.0292" x2="-1.27" y2="-5.4102" width="0.1524" layer="47"/>
<wire x1="1.27" y1="2.4892" x2="1.27" y2="-5.0292" width="0.1524" layer="47"/>
<wire x1="1.27" y1="-5.0292" x2="1.27" y2="-5.4102" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="-5.0292" x2="-2.54" y2="-5.0292" width="0.1524" layer="47"/>
<wire x1="1.27" y1="-5.0292" x2="2.54" y2="-5.0292" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="-5.0292" x2="-1.524" y2="-4.9022" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="-5.0292" x2="-1.524" y2="-5.1562" width="0.1524" layer="47"/>
<wire x1="-1.524" y1="-4.9022" x2="-1.524" y2="-5.1562" width="0.1524" layer="47"/>
<wire x1="1.27" y1="-5.0292" x2="1.524" y2="-4.9022" width="0.1524" layer="47"/>
<wire x1="1.27" y1="-5.0292" x2="1.524" y2="-5.1562" width="0.1524" layer="47"/>
<wire x1="1.524" y1="-4.9022" x2="1.524" y2="-5.1562" width="0.1524" layer="47"/>
<text x="-15.2146" y="-8.2042" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX20Y20D0T</text>
<text x="-14.8082" y="-12.7762" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-14.3002" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="2.5146" y="0.889" size="0.635" layer="47" ratio="4" rot="SR0">0.031in/0.8mm</text>
<text x="-4.0386" y="6.8072" size="0.635" layer="47" ratio="4" rot="SR0">-0.063in/-1.6mm</text>
<text x="4.4196" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.126in/3.2mm</text>
<text x="-11.2268" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.195in/4.953mm</text>
<text x="-3.175" y="-6.1722" size="0.635" layer="47" ratio="4" rot="SR0">0.1in/2.54mm</text>
<wire x1="-1.651" y1="-2.8448" x2="1.651" y2="-2.8448" width="0.1524" layer="21"/>
<wire x1="1.651" y1="-2.8448" x2="1.651" y2="2.8448" width="0.1524" layer="21"/>
<wire x1="1.651" y1="2.8448" x2="-1.651" y2="2.8448" width="0.1524" layer="21"/>
<wire x1="-1.651" y1="2.8448" x2="-1.651" y2="-2.8448" width="0.1524" layer="21"/>
<text x="0.2286" y="1.8288" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<wire x1="1.27" y1="1.2192" x2="0" y2="2.4892" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="-2.4892" x2="1.27" y2="-2.4892" width="0.1524" layer="51"/>
<wire x1="1.27" y1="-2.4892" x2="1.27" y2="2.4892" width="0.1524" layer="51"/>
<wire x1="1.27" y1="2.4892" x2="-1.27" y2="2.4892" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="2.4892" x2="-1.27" y2="-2.4892" width="0.1524" layer="51"/>
<text x="0.6858" y="0.9652" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
</packages>
<symbols>
<symbol name="VL53L1CXV0FY/1">
<pin name="AVDDVCSEL" x="2.54" y="0" length="middle" direction="pwr"/>
<pin name="AVSSVCSEL" x="2.54" y="-2.54" length="middle" direction="pwr"/>
<pin name="GND" x="2.54" y="-5.08" length="middle" direction="pwr"/>
<pin name="GND2" x="2.54" y="-7.62" length="middle" direction="pwr"/>
<pin name="XSHUT" x="2.54" y="-10.16" length="middle" direction="in"/>
<pin name="GND3" x="2.54" y="-12.7" length="middle" direction="pwr"/>
<pin name="GPIO1" x="48.26" y="-12.7" length="middle" rot="R180"/>
<pin name="DNC" x="48.26" y="-10.16" length="middle" direction="nc" rot="R180"/>
<pin name="SDA" x="48.26" y="-7.62" length="middle" rot="R180"/>
<pin name="SCL" x="48.26" y="-5.08" length="middle" direction="in" rot="R180"/>
<pin name="AVDD" x="48.26" y="-2.54" length="middle" direction="pwr" rot="R180"/>
<pin name="GND4" x="48.26" y="0" length="middle" direction="pwr" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-17.78" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-17.78" x2="43.18" y2="-17.78" width="0.1524" layer="94"/>
<wire x1="43.18" y1="-17.78" x2="43.18" y2="5.08" width="0.1524" layer="94"/>
<wire x1="43.18" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="20.6756" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="20.0406" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="VL53L1CXV0FY/1" prefix="U">
<gates>
<gate name="A" symbol="VL53L1CXV0FY/1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="LGA12_STM">
<connects>
<connect gate="A" pin="AVDD" pad="11"/>
<connect gate="A" pin="AVDDVCSEL" pad="1"/>
<connect gate="A" pin="AVSSVCSEL" pad="2"/>
<connect gate="A" pin="DNC" pad="8"/>
<connect gate="A" pin="GND" pad="3"/>
<connect gate="A" pin="GND2" pad="4"/>
<connect gate="A" pin="GND3" pad="6"/>
<connect gate="A" pin="GND4" pad="12"/>
<connect gate="A" pin="GPIO1" pad="7"/>
<connect gate="A" pin="SCL" pad="10"/>
<connect gate="A" pin="SDA" pad="9"/>
<connect gate="A" pin="XSHUT" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="VL53L1CXV0FY/1" constant="no"/>
<attribute name="MFR_NAME" value="STMicroelectronics" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="LP3985IM5X-2.8">
<packages>
<package name="MF05A">
<smd name="1" x="-1.2192" y="0.9525" dx="1.27" dy="0.5588" layer="1"/>
<smd name="2" x="-1.2192" y="0" dx="1.27" dy="0.5588" layer="1"/>
<smd name="3" x="-1.2192" y="-0.9525" dx="1.27" dy="0.5588" layer="1"/>
<smd name="4" x="1.2192" y="-0.9525" dx="1.27" dy="0.5588" layer="1"/>
<smd name="5" x="1.2192" y="0.9525" dx="1.27" dy="0.5588" layer="1"/>
<wire x1="-0.8382" y1="0.7112" x2="-0.8382" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="1.2192" x2="-1.4986" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="1.2192" x2="-1.4986" y2="0.6858" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="0.6858" x2="-0.8382" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.254" x2="-0.8382" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="0.254" x2="-1.4986" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="0.254" x2="-1.4986" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.254" x2="-0.8382" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-1.2192" x2="-0.8382" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.7112" x2="-1.4986" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.7112" x2="-1.4986" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-1.2192" x2="-0.8382" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-0.7112" x2="0.8382" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-1.2192" x2="1.4986" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="-1.2192" x2="1.4986" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="-0.7112" x2="0.8382" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="1.2192" x2="0.8382" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="0.7112" x2="1.4986" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="0.7112" x2="1.4986" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="1.2192" x2="0.8382" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-1.4986" x2="0.8382" y2="-1.4986" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-1.4986" x2="0.8382" y2="1.4986" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="1.4986" x2="-0.8382" y2="1.4986" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="1.4986" x2="-0.8382" y2="-1.4986" width="0.1524" layer="51"/>
<text x="-2.0574" y="0.8636" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<wire x1="-0.8382" y1="0" x2="-0.8382" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-0.8382" y2="3.7846" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="0" x2="0.8382" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="0.8382" y2="3.7846" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-2.1082" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="2.1082" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-1.0922" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="-1.0922" y1="3.5306" x2="-1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="1.0922" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="1.0922" y1="3.5306" x2="1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-1.4986" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.4986" y2="5.6896" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="0" x2="1.4986" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.4986" y2="5.6896" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-2.7686" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="2.7686" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.7526" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="-1.7526" y1="5.4356" x2="-1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.7526" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="1.7526" y1="5.4356" x2="1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="0" y1="1.4986" x2="3.3782" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.7592" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="0" y1="-1.4986" x2="3.3782" y2="-1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.7592" y2="-1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.3782" y2="2.7686" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.3782" y2="-2.7686" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.2512" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.5052" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="1.7526" x2="3.5052" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.2512" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.5052" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="-1.7526" x2="3.5052" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="-1.2192" y1="0.9652" x2="-3.7592" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="-3.7592" y1="0.9652" x2="-4.1402" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="-1.2192" y1="0" x2="-1.4986" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-3.7592" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.7592" y1="0" x2="-4.1402" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.7592" y1="0.9652" x2="-3.7592" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="-3.7592" y1="0" x2="-3.7592" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-3.7592" y1="0.9652" x2="-3.8862" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.7592" y1="0.9652" x2="-3.6322" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.8862" y1="1.2192" x2="-3.6322" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.7592" y1="0" x2="-3.8862" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-3.7592" y1="0" x2="-3.6322" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-3.8862" y1="-0.254" x2="-3.6322" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-1.4986" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.4986" y2="-3.7846" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="0" x2="-0.9398" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.9398" y2="-3.7846" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-2.7686" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="0.3302" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.7526" y2="-3.2766" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.7526" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-1.7526" y1="-3.2766" x2="-1.7526" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.6858" y2="-3.2766" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.6858" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-0.6858" y1="-3.2766" x2="-0.6858" y2="-3.5306" width="0.1524" layer="47"/>
<text x="-15.2146" y="-6.5786" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX50Y22D0T</text>
<text x="-14.8082" y="-8.4836" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-10.3886" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-3.7592" y="3.9116" size="0.635" layer="47" ratio="4" rot="SR0">.066in/1.676mm</text>
<text x="-3.7592" y="5.8166" size="0.635" layer="47" ratio="4" rot="SR0">.118in/2.997mm</text>
<text x="3.8862" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">.118in/2.997mm</text>
<text x="-11.7856" y="0.1524" size="0.635" layer="47" ratio="4" rot="SR0">.0375in/.953mm</text>
<text x="-4.6736" y="-4.5466" size="0.635" layer="47" ratio="4" rot="SR0">.022in/.559mm</text>
<wire x1="-0.381" y1="-1.4986" x2="0.381" y2="-1.4986" width="0.1524" layer="21"/>
<wire x1="0.8382" y1="-0.3302" x2="0.8382" y2="0.3302" width="0.1524" layer="21"/>
<wire x1="0.381" y1="1.4986" x2="-0.381" y2="1.4986" width="0.1524" layer="21"/>
<text x="-2.0574" y="0.8636" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
<package name="MF05A-M">
<smd name="1" x="-1.27" y="0.9525" dx="1.5748" dy="0.6096" layer="1"/>
<smd name="2" x="-1.27" y="0" dx="1.5748" dy="0.6096" layer="1"/>
<smd name="3" x="-1.27" y="-0.9525" dx="1.5748" dy="0.6096" layer="1"/>
<smd name="4" x="1.27" y="-0.9525" dx="1.5748" dy="0.6096" layer="1"/>
<smd name="5" x="1.27" y="0.9525" dx="1.5748" dy="0.6096" layer="1"/>
<wire x1="-0.8382" y1="0.7112" x2="-0.8382" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="1.2192" x2="-1.4986" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="1.2192" x2="-1.4986" y2="0.6858" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="0.6858" x2="-0.8382" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.254" x2="-0.8382" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="0.254" x2="-1.4986" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="0.254" x2="-1.4986" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.254" x2="-0.8382" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-1.2192" x2="-0.8382" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.7112" x2="-1.4986" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.7112" x2="-1.4986" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-1.2192" x2="-0.8382" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-0.7112" x2="0.8382" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-1.2192" x2="1.4986" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="-1.2192" x2="1.4986" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="-0.7112" x2="0.8382" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="1.2192" x2="0.8382" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="0.7112" x2="1.4986" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="0.7112" x2="1.4986" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="1.2192" x2="0.8382" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-1.4986" x2="0.8382" y2="-1.4986" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-1.4986" x2="0.8382" y2="1.4986" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="1.4986" x2="-0.8382" y2="1.4986" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="1.4986" x2="-0.8382" y2="-1.4986" width="0.1524" layer="51"/>
<text x="-1.1684" y="0.508" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<wire x1="-0.8382" y1="0" x2="-0.8382" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-0.8382" y2="3.7846" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="0" x2="0.8382" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="0.8382" y2="3.7846" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-2.1082" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="2.1082" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-1.0922" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="-1.0922" y1="3.5306" x2="-1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="1.0922" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="1.0922" y1="3.5306" x2="1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-1.4986" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.4986" y2="5.6896" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="0" x2="1.4986" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.4986" y2="5.6896" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-2.7686" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="2.7686" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.7526" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="-1.7526" y1="5.4356" x2="-1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.7526" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="1.7526" y1="5.4356" x2="1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="0" y1="1.4986" x2="3.3782" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.7592" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="0" y1="-1.4986" x2="3.3782" y2="-1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.7592" y2="-1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.3782" y2="2.7686" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.3782" y2="-2.7686" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.2512" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.5052" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="1.7526" x2="3.5052" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.2512" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.5052" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="-1.7526" x2="3.5052" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="0.9652" x2="-3.81" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="-3.81" y1="0.9652" x2="-4.191" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="0" x2="-1.4986" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-3.81" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.81" y1="0" x2="-4.191" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.81" y1="0.9652" x2="-3.81" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="-3.81" y1="0" x2="-3.81" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-3.81" y1="0.9652" x2="-3.937" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.81" y1="0.9652" x2="-3.683" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.937" y1="1.2192" x2="-3.683" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.81" y1="0" x2="-3.937" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-3.81" y1="0" x2="-3.683" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-3.937" y1="-0.254" x2="-3.683" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-1.4986" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.4986" y2="-3.7846" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="0" x2="-0.9398" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.9398" y2="-3.7846" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-2.7686" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="0.3302" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.7526" y2="-3.2766" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.7526" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-1.7526" y1="-3.2766" x2="-1.7526" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.6858" y2="-3.2766" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.6858" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-0.6858" y1="-3.2766" x2="-0.6858" y2="-3.5306" width="0.1524" layer="47"/>
<text x="-15.2146" y="-6.5786" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX62Y24D0T</text>
<text x="-14.8082" y="-9.6266" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-11.1506" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-4.0386" y="3.9116" size="0.635" layer="47" ratio="4" rot="SR0">0.066in/1.676mm</text>
<text x="-4.0386" y="5.8166" size="0.635" layer="47" ratio="4" rot="SR0">0.118in/2.997mm</text>
<text x="3.8862" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.118in/2.997mm</text>
<text x="-12.3952" y="0.1524" size="0.635" layer="47" ratio="4" rot="SR0">0.038in/0.952mm</text>
<text x="-5.2578" y="-4.5466" size="0.635" layer="47" ratio="4" rot="SR0">0.022in/0.559mm</text>
<wire x1="-0.9652" y1="-1.6256" x2="0.9652" y2="-1.6256" width="0.1524" layer="21"/>
<wire x1="0.9652" y1="-0.3048" x2="0.9652" y2="0.3048" width="0.1524" layer="21"/>
<wire x1="0.9652" y1="1.6256" x2="-0.9652" y2="1.6256" width="0.1524" layer="21"/>
<text x="-2.1082" y="1.016" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
<package name="MF05A-L">
<smd name="1" x="-1.1684" y="0.9525" dx="0.9652" dy="0.508" layer="1"/>
<smd name="2" x="-1.1684" y="0" dx="0.9652" dy="0.508" layer="1"/>
<smd name="3" x="-1.1684" y="-0.9525" dx="0.9652" dy="0.508" layer="1"/>
<smd name="4" x="1.1684" y="-0.9525" dx="0.9652" dy="0.508" layer="1"/>
<smd name="5" x="1.1684" y="0.9525" dx="0.9652" dy="0.508" layer="1"/>
<wire x1="-0.8382" y1="0.7112" x2="-0.8382" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="1.2192" x2="-1.4986" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="1.2192" x2="-1.4986" y2="0.6858" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="0.6858" x2="-0.8382" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.254" x2="-0.8382" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="0.254" x2="-1.4986" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="0.254" x2="-1.4986" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.254" x2="-0.8382" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-1.2192" x2="-0.8382" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-0.7112" x2="-1.4986" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-0.7112" x2="-1.4986" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="-1.4986" y1="-1.2192" x2="-0.8382" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-0.7112" x2="0.8382" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-1.2192" x2="1.4986" y2="-1.2192" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="-1.2192" x2="1.4986" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="-0.7112" x2="0.8382" y2="-0.7112" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="1.2192" x2="0.8382" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="0.7112" x2="1.4986" y2="0.7112" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="0.7112" x2="1.4986" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="1.4986" y1="1.2192" x2="0.8382" y2="1.2192" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="-1.4986" x2="0.8382" y2="-1.4986" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="-1.4986" x2="0.8382" y2="1.4986" width="0.1524" layer="51"/>
<wire x1="0.8382" y1="1.4986" x2="-0.8382" y2="1.4986" width="0.1524" layer="51"/>
<wire x1="-0.8382" y1="1.4986" x2="-0.8382" y2="-1.4986" width="0.1524" layer="51"/>
<text x="-1.1684" y="0.4572" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<wire x1="-0.8382" y1="0" x2="-0.8382" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-0.8382" y2="3.7846" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="0" x2="0.8382" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="0.8382" y2="3.7846" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-2.1082" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="2.1082" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-1.0922" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="-0.8382" y1="3.4036" x2="-1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="-1.0922" y1="3.5306" x2="-1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="1.0922" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.8382" y1="3.4036" x2="1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="1.0922" y1="3.5306" x2="1.0922" y2="3.2766" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-1.4986" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.4986" y2="5.6896" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="0" x2="1.4986" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.4986" y2="5.6896" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-2.7686" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="2.7686" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.7526" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="5.3086" x2="-1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="-1.7526" y1="5.4356" x2="-1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.7526" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="1.4986" y1="5.3086" x2="1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="1.7526" y1="5.4356" x2="1.7526" y2="5.1816" width="0.1524" layer="47"/>
<wire x1="0" y1="1.4986" x2="3.3782" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.7592" y2="1.4986" width="0.1524" layer="47"/>
<wire x1="0" y1="-1.4986" x2="3.3782" y2="-1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.7592" y2="-1.4986" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.3782" y2="2.7686" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.3782" y2="-2.7686" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.2512" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="1.4986" x2="3.5052" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="1.7526" x2="3.5052" y2="1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.2512" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="3.3782" y1="-1.4986" x2="3.5052" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="3.2512" y1="-1.7526" x2="3.5052" y2="-1.7526" width="0.1524" layer="47"/>
<wire x1="-1.1684" y1="0.9652" x2="-3.7084" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="-3.7084" y1="0.9652" x2="-4.0894" y2="0.9652" width="0.1524" layer="47"/>
<wire x1="-1.1684" y1="0" x2="-1.4986" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-3.7084" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.7084" y1="0" x2="-4.0894" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.7084" y1="0.9652" x2="-3.7084" y2="2.2352" width="0.1524" layer="47"/>
<wire x1="-3.7084" y1="0" x2="-3.7084" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-3.7084" y1="0.9652" x2="-3.8354" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.7084" y1="0.9652" x2="-3.5814" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.8354" y1="1.2192" x2="-3.5814" y2="1.2192" width="0.1524" layer="47"/>
<wire x1="-3.7084" y1="0" x2="-3.8354" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-3.7084" y1="0" x2="-3.5814" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-3.8354" y1="-0.254" x2="-3.5814" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="0" x2="-1.4986" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.4986" y2="-3.7846" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="0" x2="-0.9398" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.9398" y2="-3.7846" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-2.7686" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="0.3302" y2="-3.4036" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.7526" y2="-3.2766" width="0.1524" layer="47"/>
<wire x1="-1.4986" y1="-3.4036" x2="-1.7526" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-1.7526" y1="-3.2766" x2="-1.7526" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.6858" y2="-3.2766" width="0.1524" layer="47"/>
<wire x1="-0.9398" y1="-3.4036" x2="-0.6858" y2="-3.5306" width="0.1524" layer="47"/>
<wire x1="-0.6858" y1="-3.2766" x2="-0.6858" y2="-3.5306" width="0.1524" layer="47"/>
<text x="-15.2146" y="-6.5786" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX38Y20D0T</text>
<text x="-14.8082" y="-9.6266" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-11.1506" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-4.0386" y="3.9116" size="0.635" layer="47" ratio="4" rot="SR0">0.066in/1.676mm</text>
<text x="-4.0386" y="5.8166" size="0.635" layer="47" ratio="4" rot="SR0">0.118in/2.997mm</text>
<text x="3.8862" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.118in/2.997mm</text>
<text x="-12.2936" y="0.1524" size="0.635" layer="47" ratio="4" rot="SR0">0.038in/0.952mm</text>
<text x="-5.2578" y="-4.5466" size="0.635" layer="47" ratio="4" rot="SR0">0.022in/0.559mm</text>
<wire x1="-0.9652" y1="-1.6256" x2="0.9652" y2="-1.6256" width="0.1524" layer="21"/>
<wire x1="0.9652" y1="-0.3556" x2="0.9652" y2="0.3556" width="0.1524" layer="21"/>
<wire x1="0.9652" y1="1.6256" x2="-0.9652" y2="1.6256" width="0.1524" layer="21"/>
<text x="-2.0066" y="0.9652" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
</packages>
<symbols>
<symbol name="LP3985IM5-2.5">
<pin name="VIN" x="2.54" y="0" length="middle" direction="in"/>
<pin name="GND" x="2.54" y="-2.54" length="middle" direction="pas"/>
<pin name="VEN" x="38.1" y="0" length="middle" direction="pas" rot="R180"/>
<pin name="BP" x="38.1" y="-2.54" length="middle" direction="pas" rot="R180"/>
<pin name="VOUT" x="38.1" y="-5.08" length="middle" direction="out" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-10.16" x2="33.02" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="33.02" y1="-10.16" x2="33.02" y2="5.08" width="0.1524" layer="94"/>
<wire x1="33.02" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="15.5956" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="14.9606" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="LP3985IM5X-2.8/NOPB" prefix="U">
<gates>
<gate name="A" symbol="LP3985IM5-2.5" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MF05A">
<connects>
<connect gate="A" pin="BP" pad="4"/>
<connect gate="A" pin="GND" pad="2"/>
<connect gate="A" pin="VEN" pad="3"/>
<connect gate="A" pin="VIN" pad="1"/>
<connect gate="A" pin="VOUT" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="LP3985IM5X-2.8/NOPB" constant="no"/>
<attribute name="MFR_NAME" value="Texas Instruments" constant="no"/>
<attribute name="TYPE" value="LP3985IM5-2.5" constant="no"/>
</technology>
</technologies>
</device>
<device name="MF05A-M" package="MF05A-M">
<connects>
<connect gate="A" pin="BP" pad="4"/>
<connect gate="A" pin="GND" pad="2"/>
<connect gate="A" pin="VEN" pad="3"/>
<connect gate="A" pin="VIN" pad="1"/>
<connect gate="A" pin="VOUT" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="LP3985IM5X-2.8/NOPB" constant="no"/>
<attribute name="MFR_NAME" value="Texas Instruments" constant="no"/>
<attribute name="TYPE" value="LP3985IM5-2.5" constant="no"/>
</technology>
</technologies>
</device>
<device name="MF05A-L" package="MF05A-L">
<connects>
<connect gate="A" pin="BP" pad="4"/>
<connect gate="A" pin="GND" pad="2"/>
<connect gate="A" pin="VEN" pad="3"/>
<connect gate="A" pin="VIN" pad="1"/>
<connect gate="A" pin="VOUT" pad="5"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="LP3985IM5X-2.8/NOPB" constant="no"/>
<attribute name="MFR_NAME" value="Texas Instruments" constant="no"/>
<attribute name="TYPE" value="LP3985IM5-2.5" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="MCM3400A-TP">
<packages>
<package name="DFN2020-6L_MCE">
<smd name="1" x="-0.9906" y="0.6499875" dx="0.3048" dy="0.6858" layer="1" rot="R270"/>
<smd name="2" x="-0.9906" y="0" dx="0.3048" dy="0.6858" layer="1" rot="R270"/>
<smd name="3" x="-0.9906" y="-0.6499875" dx="0.3048" dy="0.6858" layer="1" rot="R270"/>
<smd name="4" x="0.9906" y="-0.6499875" dx="0.3048" dy="0.6858" layer="1" rot="R90"/>
<smd name="5" x="0.9906" y="0" dx="0.3048" dy="0.6858" layer="1" rot="R90"/>
<smd name="6" x="0.9906" y="0.6499875" dx="0.3048" dy="0.6858" layer="1" rot="R90"/>
<polygon width="0.0254" layer="29">
<vertex x="-0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.1778"/>
<vertex x="-0.5461" y="0.1778"/>
</polygon>
<polygon width="0.0254" layer="29">
<vertex x="-0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.8636"/>
<vertex x="-0.5461" y="-0.8636"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.1778"/>
<vertex x="-0.5461" y="0.1778"/>
</polygon>
<polygon width="0.0254" layer="31">
<vertex x="-0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.8636"/>
<vertex x="-0.5461" y="-0.8636"/>
</polygon>
<wire x1="0.9906" y1="0.6604" x2="1.016" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="1.016" y1="0.6604" x2="4.9784" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="4.9784" y1="0.6604" x2="5.3848" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="2.5908" y1="0" x2="4.9784" y2="0" width="0.1524" layer="47"/>
<wire x1="4.9784" y1="0" x2="5.3848" y2="0" width="0.1524" layer="47"/>
<wire x1="4.9784" y1="0.6604" x2="4.9784" y2="1.9304" width="0.1524" layer="47"/>
<wire x1="4.9784" y1="0" x2="4.9784" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="4.9784" y1="0.6604" x2="4.8768" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="4.9784" y1="0.6604" x2="5.1308" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="4.8768" y1="0.9144" x2="5.1308" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="4.9784" y1="0" x2="4.8768" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="4.9784" y1="0" x2="5.1308" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="4.8768" y1="-0.254" x2="5.1308" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="0.6604" x2="0.6604" y2="2.54" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.6604" y2="2.921" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.016" y2="2.921" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="-0.6096" y2="2.54" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="2.286" y2="2.54" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.4064" y2="2.667" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.4064" y2="2.413" width="0.1524" layer="47"/>
<wire x1="0.4064" y1="2.667" x2="0.4064" y2="2.413" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.27" y2="2.667" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.27" y2="2.413" width="0.1524" layer="47"/>
<wire x1="1.27" y1="2.667" x2="1.27" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.016" y2="1.778" width="0.1524" layer="47"/>
<wire x1="1.016" y1="0.6604" x2="1.016" y2="1.016" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.016" x2="1.016" y2="1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.016" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-2.286" y2="1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="2.286" y2="1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.27" y2="1.524" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="1.524" x2="-1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.27" y2="1.524" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="1.27" y1="1.524" x2="1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="0" x2="2.5908" y2="0" width="0.1524" layer="47"/>
<wire x1="2.5908" y1="0" x2="2.5908" y2="1.27" width="0.1524" layer="47"/>
<wire x1="2.5908" y1="0" x2="2.5908" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="2.5908" y1="0" x2="2.4384" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.5908" y1="0" x2="2.6924" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.4384" y1="0.254" x2="2.6924" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.5908" y1="0" x2="2.4384" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="2.5908" y1="0" x2="2.6924" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="2.4384" y1="-0.254" x2="2.6924" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.016" x2="-1.016" y2="1.016" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-2.5908" y2="1.016" width="0.1524" layer="47"/>
<wire x1="-2.5908" y1="1.016" x2="-2.9464" y2="1.016" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.016" x2="-2.5908" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="-2.5908" y1="-1.016" x2="-2.9464" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="-2.5908" y1="1.016" x2="-2.5908" y2="2.286" width="0.1524" layer="47"/>
<wire x1="-2.5908" y1="-1.016" x2="-2.5908" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="-2.5908" y1="1.016" x2="-2.6924" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.5908" y1="1.016" x2="-2.4384" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.6924" y1="1.27" x2="-2.4384" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.5908" y1="-1.016" x2="-2.6924" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.5908" y1="-1.016" x2="-2.4384" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.6924" y1="-1.27" x2="-2.4384" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.016" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="1.016" y1="0.6604" x2="1.016" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.016" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-2.286" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="2.286" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.27" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.27" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.1778" x2="-0.5588" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.8636" x2="-0.5588" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.5588" y2="4.572" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="0.1778" x2="0.5588" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.5588" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-1.8288" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="1.8288" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.8128" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="-0.8128" y1="4.318" x2="-0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.8128" y2="4.318" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="0.8128" y1="4.318" x2="0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.8636" x2="-4.2164" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.572" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.1778" x2="-4.2164" y2="0.1778" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.572" y2="0.1778" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.2164" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.2164" y2="-1.0922" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.318" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.064" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.318" y1="1.1176" x2="-4.064" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.318" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.064" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-4.318" y1="-0.0762" x2="-4.064" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.8636" x2="-0.5588" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.1778" x2="-0.5588" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.5588" y2="6.35" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="-0.8636" x2="0.5588" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.5588" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-1.8288" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="1.8288" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.8128" y2="6.096" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="-0.8128" y1="6.096" x2="-0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.8128" y2="6.096" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="0.8128" y1="6.096" x2="0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.1778" x2="-10.0584" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.414" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.8636" x2="-10.0584" y2="-0.8636" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.414" y2="-0.8636" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.0584" y2="1.0922" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.0584" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.16" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-9.906" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.16" y1="0.0762" x2="-9.906" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.16" y2="-1.1176" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-9.906" y2="-1.1176" width="0.1524" layer="47"/>
<wire x1="-10.16" y1="-1.1176" x2="-9.906" y2="-1.1176" width="0.1524" layer="47"/>
<text x="-15.2146" y="-6.35" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX14Y15D0T</text>
<text x="-17.2974" y="-7.874" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab 1 Padstyle: RX43Y27D0T</text>
<text x="-17.2974" y="-9.398" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab 2 Padstyle: RX43Y27D0T</text>
<text x="-14.8082" y="-10.922" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-12.446" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="1.2446" y="0" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.65mm</text>
<text x="-3.2004" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.015in/0.381mm</text>
<text x="-4.0386" y="1.905" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="3.0988" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0in/0mm</text>
<text x="-11.176" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="-4.0386" y="-2.54" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="-4.0386" y="4.7244" size="0.635" layer="47" ratio="4" rot="SR0">0.043in/1.092mm</text>
<text x="-12.8016" y="0.2032" size="0.635" layer="47" ratio="4" rot="SR0">0.027in/0.686mm</text>
<text x="-4.0386" y="6.5024" size="0.635" layer="47" ratio="4" rot="SR0">0.043in/1.092mm</text>
<text x="-18.6436" y="-0.8382" size="0.635" layer="47" ratio="4" rot="SR0">0.027in/0.686mm</text>
<wire x1="-1.1684" y1="-1.1684" x2="-0.7112" y2="-1.1684" width="0.1524" layer="21"/>
<wire x1="1.1684" y1="1.1684" x2="0.7112" y2="1.1684" width="0.1524" layer="21"/>
<wire x1="-0.7112" y1="1.1684" x2="-1.1684" y2="1.1684" width="0.1524" layer="21"/>
<wire x1="0.7112" y1="-1.1684" x2="1.1684" y2="-1.1684" width="0.1524" layer="21"/>
<text x="-2.6924" y="0.0254" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<wire x1="-1.016" y1="-0.254" x2="0.254" y2="1.016" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="0.8382" x2="-1.016" y2="0.4826" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="0.1778" x2="-1.016" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="-0.4826" x2="-1.016" y2="-0.8382" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-0.8382" x2="1.016" y2="-0.4826" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-0.1778" x2="1.016" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="1.016" y1="0.4826" x2="1.016" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="-1.016" x2="1.016" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-1.016" x2="1.016" y2="1.016" width="0.1524" layer="51"/>
<wire x1="1.016" y1="1.016" x2="-1.016" y2="1.016" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="-1.016" width="0.1524" layer="51"/>
<text x="-0.4318" y="0.0254" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
<smd name="P$1" x="0" y="-0.5842" dx="0.84" dy="0.44" layer="1"/>
<smd name="P$2" x="0" y="0.5842" dx="0.84" dy="0.44" layer="1"/>
</package>
<package name="DFN2020-6L_MCE-M">
<smd name="1" x="-1.0414" y="0.6499875" dx="0.3048" dy="0.7874" layer="1" rot="R270"/>
<smd name="2" x="-1.0414" y="0" dx="0.3048" dy="0.7874" layer="1" rot="R270"/>
<smd name="3" x="-1.0414" y="-0.6499875" dx="0.3048" dy="0.7874" layer="1" rot="R270"/>
<smd name="4" x="1.0414" y="-0.6499875" dx="0.3048" dy="0.7874" layer="1" rot="R90"/>
<smd name="5" x="1.0414" y="0" dx="0.3048" dy="0.7874" layer="1" rot="R90"/>
<smd name="6" x="1.0414" y="0.6499875" dx="0.3048" dy="0.7874" layer="1" rot="R90"/>
<polygon width="0.1524" layer="1">
<vertex x="-0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.1778"/>
<vertex x="-0.5461" y="0.1778"/>
</polygon>
<polygon width="0.1524" layer="1">
<vertex x="-0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.8636"/>
<vertex x="-0.5461" y="-0.8636"/>
</polygon>
<polygon width="0.1524" layer="29">
<vertex x="-0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.1778"/>
<vertex x="-0.5461" y="0.1778"/>
</polygon>
<polygon width="0.1524" layer="29">
<vertex x="-0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.8636"/>
<vertex x="-0.5461" y="-0.8636"/>
</polygon>
<polygon width="0.1524" layer="31">
<vertex x="-0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.1778"/>
<vertex x="-0.5461" y="0.1778"/>
</polygon>
<polygon width="0.1524" layer="31">
<vertex x="-0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.8636"/>
<vertex x="-0.5461" y="-0.8636"/>
</polygon>
<wire x1="1.0414" y1="0.6604" x2="5.0292" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="5.0292" y1="0.6604" x2="5.4356" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="2.6416" y1="0" x2="5.0292" y2="0" width="0.1524" layer="47"/>
<wire x1="5.0292" y1="0" x2="5.4356" y2="0" width="0.1524" layer="47"/>
<wire x1="5.0292" y1="0.6604" x2="5.0292" y2="1.9304" width="0.1524" layer="47"/>
<wire x1="5.0292" y1="0" x2="5.0292" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="5.0292" y1="0.6604" x2="4.9276" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="5.0292" y1="0.6604" x2="5.1816" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0.9144" x2="5.1816" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="5.0292" y1="0" x2="4.9276" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="5.0292" y1="0" x2="5.1816" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="-0.254" x2="5.1816" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="0.6604" x2="0.6604" y2="2.54" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.6604" y2="2.921" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.016" y2="2.921" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="-0.6096" y2="2.54" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="2.286" y2="2.54" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.4064" y2="2.667" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.4064" y2="2.413" width="0.1524" layer="47"/>
<wire x1="0.4064" y1="2.667" x2="0.4064" y2="2.413" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.27" y2="2.667" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.27" y2="2.413" width="0.1524" layer="47"/>
<wire x1="1.27" y1="2.667" x2="1.27" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.016" y2="1.778" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.016" x2="1.016" y2="1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.016" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-2.286" y2="1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="2.286" y2="1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.27" y2="1.524" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="1.524" x2="-1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.27" y2="1.524" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="1.27" y1="1.524" x2="1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="0" x2="2.6416" y2="0" width="0.1524" layer="47"/>
<wire x1="2.6416" y1="0" x2="2.6416" y2="1.27" width="0.1524" layer="47"/>
<wire x1="2.6416" y1="0" x2="2.6416" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="2.6416" y1="0" x2="2.4892" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.6416" y1="0" x2="2.7432" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="0.254" x2="2.7432" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.6416" y1="0" x2="2.4892" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="2.6416" y1="0" x2="2.7432" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="2.4892" y1="-0.254" x2="2.7432" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.016" x2="-1.016" y2="1.016" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-2.6416" y2="1.016" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="1.016" x2="-2.9972" y2="1.016" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.016" x2="-2.6416" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="-1.016" x2="-2.9972" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="1.016" x2="-2.6416" y2="2.286" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="-1.016" x2="-2.6416" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="1.016" x2="-2.7432" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="1.016" x2="-2.4892" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="1.27" x2="-2.4892" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="-1.016" x2="-2.7432" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="-1.016" x2="-2.4892" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.7432" y1="-1.27" x2="-2.4892" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.016" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.016" x2="1.016" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.016" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-2.286" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="2.286" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.27" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.27" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.1778" x2="-0.5588" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.8636" x2="-0.5588" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.5588" y2="4.572" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="0.1778" x2="0.5588" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.5588" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-1.8288" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="1.8288" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.8128" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="-0.8128" y1="4.318" x2="-0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.8128" y2="4.318" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="0.8128" y1="4.318" x2="0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.8636" x2="-4.2164" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.572" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.1778" x2="-4.2164" y2="0.1778" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.572" y2="0.1778" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.2164" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.2164" y2="-1.0922" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.318" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.064" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.318" y1="1.1176" x2="-4.064" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.318" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.064" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-4.318" y1="-0.0762" x2="-4.064" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.8636" x2="-0.5588" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.1778" x2="-0.5588" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.5588" y2="6.35" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="-0.8636" x2="0.5588" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.5588" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-1.8288" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="1.8288" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.8128" y2="6.096" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="-0.8128" y1="6.096" x2="-0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.8128" y2="6.096" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="0.8128" y1="6.096" x2="0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.1778" x2="-10.0584" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.414" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.8636" x2="-10.0584" y2="-0.8636" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.414" y2="-0.8636" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.0584" y2="1.0922" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.0584" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.16" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-9.906" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.16" y1="0.0762" x2="-9.906" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.16" y2="-1.1176" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-9.906" y2="-1.1176" width="0.1524" layer="47"/>
<wire x1="-10.16" y1="-1.1176" x2="-9.906" y2="-1.1176" width="0.1524" layer="47"/>
<text x="-15.2146" y="-6.35" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX16Y19D0T</text>
<text x="-17.2974" y="-7.874" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab 1 Padstyle: RX43Y27D0T</text>
<text x="-17.2974" y="-9.398" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab 2 Padstyle: RX43Y27D0T</text>
<text x="-14.8082" y="-10.922" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-12.446" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="1.2954" y="0" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.65mm</text>
<text x="-3.2004" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.015in/0.381mm</text>
<text x="-4.0386" y="1.905" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="3.1496" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0in/0mm</text>
<text x="-11.2268" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="-4.0386" y="-2.54" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="-4.0386" y="4.7244" size="0.635" layer="47" ratio="4" rot="SR0">0.043in/1.092mm</text>
<text x="-12.8016" y="0.2032" size="0.635" layer="47" ratio="4" rot="SR0">0.027in/0.686mm</text>
<text x="-4.0386" y="6.5024" size="0.635" layer="47" ratio="4" rot="SR0">0.043in/1.092mm</text>
<text x="-18.6436" y="-0.8382" size="0.635" layer="47" ratio="4" rot="SR0">0.027in/0.686mm</text>
<wire x1="-1.1684" y1="-1.1684" x2="-0.7112" y2="-1.1684" width="0.1524" layer="21"/>
<wire x1="1.1684" y1="1.1684" x2="0.7112" y2="1.1684" width="0.1524" layer="21"/>
<wire x1="-0.7112" y1="1.1684" x2="-1.1684" y2="1.1684" width="0.1524" layer="21"/>
<wire x1="0.7112" y1="-1.1684" x2="1.1684" y2="-1.1684" width="0.1524" layer="21"/>
<text x="-2.794" y="0.0254" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<wire x1="-1.016" y1="-0.254" x2="0.254" y2="1.016" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="0.8382" x2="-1.016" y2="0.4826" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="0.1778" x2="-1.016" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="-0.4826" x2="-1.016" y2="-0.8382" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-0.8382" x2="1.016" y2="-0.4826" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-0.1778" x2="1.016" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="1.016" y1="0.4826" x2="1.016" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="-1.016" x2="1.016" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-1.016" x2="1.016" y2="1.016" width="0.1524" layer="51"/>
<wire x1="1.016" y1="1.016" x2="-1.016" y2="1.016" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="-1.016" width="0.1524" layer="51"/>
<text x="-0.4318" y="0.0254" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
<package name="DFN2020-6L_MCE-L">
<smd name="1" x="-0.9398" y="0.6499875" dx="0.3048" dy="0.5842" layer="1" rot="R270"/>
<smd name="2" x="-0.9398" y="0" dx="0.3048" dy="0.5842" layer="1" rot="R270"/>
<smd name="3" x="-0.9398" y="-0.6499875" dx="0.3048" dy="0.5842" layer="1" rot="R270"/>
<smd name="4" x="0.9398" y="-0.6499875" dx="0.3048" dy="0.5842" layer="1" rot="R90"/>
<smd name="5" x="0.9398" y="0" dx="0.3048" dy="0.5842" layer="1" rot="R90"/>
<smd name="6" x="0.9398" y="0.6499875" dx="0.3048" dy="0.5842" layer="1" rot="R90"/>
<polygon width="0.1524" layer="1">
<vertex x="-0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.1778"/>
<vertex x="-0.5461" y="0.1778"/>
</polygon>
<polygon width="0.1524" layer="1">
<vertex x="-0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.8636"/>
<vertex x="-0.5461" y="-0.8636"/>
</polygon>
<polygon width="0.1524" layer="29">
<vertex x="-0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.1778"/>
<vertex x="-0.5461" y="0.1778"/>
</polygon>
<polygon width="0.1524" layer="29">
<vertex x="-0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.8636"/>
<vertex x="-0.5461" y="-0.8636"/>
</polygon>
<polygon width="0.1524" layer="31">
<vertex x="-0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.8636"/>
<vertex x="0.5461" y="0.1778"/>
<vertex x="-0.5461" y="0.1778"/>
</polygon>
<polygon width="0.1524" layer="31">
<vertex x="-0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.1778"/>
<vertex x="0.5461" y="-0.8636"/>
<vertex x="-0.5461" y="-0.8636"/>
</polygon>
<wire x1="0.9398" y1="0.6604" x2="1.016" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="1.016" y1="0.6604" x2="4.9276" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0.6604" x2="5.334" y2="0.6604" width="0.1524" layer="47"/>
<wire x1="2.54" y1="0" x2="4.9276" y2="0" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0" x2="5.334" y2="0" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0.6604" x2="4.9276" y2="1.9304" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0" x2="4.9276" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0.6604" x2="4.826" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0.6604" x2="5.08" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="4.826" y1="0.9144" x2="5.08" y2="0.9144" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0" x2="4.826" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="4.9276" y1="0" x2="5.08" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="4.826" y1="-0.254" x2="5.08" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="0.6604" x2="0.6604" y2="2.54" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.6604" y2="2.921" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.016" y2="2.921" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="-0.6096" y2="2.54" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="2.286" y2="2.54" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.4064" y2="2.667" width="0.1524" layer="47"/>
<wire x1="0.6604" y1="2.54" x2="0.4064" y2="2.413" width="0.1524" layer="47"/>
<wire x1="0.4064" y1="2.667" x2="0.4064" y2="2.413" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.27" y2="2.667" width="0.1524" layer="47"/>
<wire x1="1.016" y1="2.54" x2="1.27" y2="2.413" width="0.1524" layer="47"/>
<wire x1="1.27" y1="2.667" x2="1.27" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.016" y2="1.778" width="0.1524" layer="47"/>
<wire x1="1.016" y1="0.6604" x2="1.016" y2="1.016" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.016" x2="1.016" y2="1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.016" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-2.286" y2="1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="2.286" y2="1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.27" y2="1.524" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.397" x2="-1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="1.524" x2="-1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.27" y2="1.524" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.397" x2="1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="1.27" y1="1.524" x2="1.27" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-0.3302" y1="0" x2="2.54" y2="0" width="0.1524" layer="47"/>
<wire x1="2.54" y1="0" x2="2.54" y2="1.27" width="0.1524" layer="47"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="2.54" y1="0" x2="2.3876" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.54" y1="0" x2="2.6416" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.3876" y1="0.254" x2="2.6416" y2="0.254" width="0.1524" layer="47"/>
<wire x1="2.54" y1="0" x2="2.3876" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="2.54" y1="0" x2="2.6416" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="2.3876" y1="-0.254" x2="2.6416" y2="-0.254" width="0.1524" layer="47"/>
<wire x1="1.016" y1="1.016" x2="-1.016" y2="1.016" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-2.54" y2="1.016" width="0.1524" layer="47"/>
<wire x1="-2.54" y1="1.016" x2="-2.8956" y2="1.016" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.016" x2="-2.54" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="-2.54" y1="-1.016" x2="-2.8956" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="-2.54" y1="1.016" x2="-2.54" y2="2.286" width="0.1524" layer="47"/>
<wire x1="-2.54" y1="-1.016" x2="-2.54" y2="-2.286" width="0.1524" layer="47"/>
<wire x1="-2.54" y1="1.016" x2="-2.6416" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.54" y1="1.016" x2="-2.3876" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="1.27" x2="-2.3876" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-2.54" y1="-1.016" x2="-2.6416" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.54" y1="-1.016" x2="-2.3876" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-2.6416" y1="-1.27" x2="-2.3876" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.016" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="1.016" y1="0.6604" x2="1.016" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.016" y2="-1.778" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-2.286" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="2.286" y2="-1.397" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.27" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="-1.016" y1="-1.397" x2="-1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.27" y2="-1.27" width="0.1524" layer="47"/>
<wire x1="1.016" y1="-1.397" x2="1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-1.524" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.1778" x2="-0.5588" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.8636" x2="-0.5588" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.5588" y2="4.572" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="0.1778" x2="0.5588" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.5588" y2="4.572" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-1.8288" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="1.8288" y2="4.2164" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.8128" y2="4.318" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="4.2164" x2="-0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="-0.8128" y1="4.318" x2="-0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.8128" y2="4.318" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="4.2164" x2="0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="0.8128" y1="4.318" x2="0.8128" y2="4.064" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.8636" x2="-4.2164" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.572" y2="0.8636" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="0.1778" x2="-4.2164" y2="0.1778" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.572" y2="0.1778" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.2164" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.2164" y2="-1.0922" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.318" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.8636" x2="-4.064" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.318" y1="1.1176" x2="-4.064" y2="1.1176" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.318" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-4.2164" y1="0.1778" x2="-4.064" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-4.318" y1="-0.0762" x2="-4.064" y2="-0.0762" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.8636" x2="-0.5588" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.1778" x2="-0.5588" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.5588" y2="6.35" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="-0.8636" x2="0.5588" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.5588" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-1.8288" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="1.8288" y2="5.9944" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.8128" y2="6.096" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="5.9944" x2="-0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="-0.8128" y1="6.096" x2="-0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.8128" y2="6.096" width="0.1524" layer="47"/>
<wire x1="0.5588" y1="5.9944" x2="0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="0.8128" y1="6.096" x2="0.8128" y2="5.842" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.1778" x2="-10.0584" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.414" y2="-0.1778" width="0.1524" layer="47"/>
<wire x1="-0.5588" y1="-0.8636" x2="-10.0584" y2="-0.8636" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.414" y2="-0.8636" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.0584" y2="1.0922" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.0584" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-10.16" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.1778" x2="-9.906" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.16" y1="0.0762" x2="-9.906" y2="0.0762" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-10.16" y2="-1.1176" width="0.1524" layer="47"/>
<wire x1="-10.0584" y1="-0.8636" x2="-9.906" y2="-1.1176" width="0.1524" layer="47"/>
<wire x1="-10.16" y1="-1.1176" x2="-9.906" y2="-1.1176" width="0.1524" layer="47"/>
<text x="-15.2146" y="-6.35" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX12Y11D0T</text>
<text x="-17.2974" y="-7.874" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab 1 Padstyle: RX43Y27D0T</text>
<text x="-17.2974" y="-9.398" size="1.27" layer="47" ratio="6" rot="SR0">Heat Tab 2 Padstyle: RX43Y27D0T</text>
<text x="-14.8082" y="-10.922" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-12.446" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="1.1938" y="0" size="0.635" layer="47" ratio="4" rot="SR0">0.026in/0.65mm</text>
<text x="-3.2004" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.015in/0.381mm</text>
<text x="-4.0386" y="1.905" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="3.048" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0in/0mm</text>
<text x="-11.1252" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="-4.0386" y="-2.54" size="0.635" layer="47" ratio="4" rot="SR0">0.081in/2.057mm</text>
<text x="-4.0386" y="4.7244" size="0.635" layer="47" ratio="4" rot="SR0">0.043in/1.092mm</text>
<text x="-12.8016" y="0.2032" size="0.635" layer="47" ratio="4" rot="SR0">0.027in/0.686mm</text>
<text x="-4.0386" y="6.5024" size="0.635" layer="47" ratio="4" rot="SR0">0.043in/1.092mm</text>
<text x="-18.6436" y="-0.8382" size="0.635" layer="47" ratio="4" rot="SR0">0.027in/0.686mm</text>
<wire x1="-1.1684" y1="-1.1684" x2="-0.7112" y2="-1.1684" width="0.1524" layer="21"/>
<wire x1="1.1684" y1="1.1684" x2="0.7112" y2="1.1684" width="0.1524" layer="21"/>
<wire x1="-0.7112" y1="1.1684" x2="-1.1684" y2="1.1684" width="0.1524" layer="21"/>
<wire x1="0.7112" y1="-1.1684" x2="1.1684" y2="-1.1684" width="0.1524" layer="21"/>
<text x="-2.5908" y="0.0254" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<wire x1="-1.016" y1="-0.254" x2="0.254" y2="1.016" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="0.8382" x2="-1.016" y2="0.4826" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="0.1778" x2="-1.016" y2="-0.1778" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="-0.4826" x2="-1.016" y2="-0.8382" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-0.8382" x2="1.016" y2="-0.4826" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-0.1778" x2="1.016" y2="0.1778" width="0.1524" layer="51"/>
<wire x1="1.016" y1="0.4826" x2="1.016" y2="0.8382" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="-1.016" x2="1.016" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-1.016" x2="1.016" y2="1.016" width="0.1524" layer="51"/>
<wire x1="1.016" y1="1.016" x2="-1.016" y2="1.016" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="1.016" x2="-1.016" y2="-1.016" width="0.1524" layer="51"/>
<text x="-0.4318" y="0.0254" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
</packages>
<symbols>
<symbol name="DFN2020-6L">
<pin name="1" x="2.54" y="0" visible="pad" length="middle" direction="pas"/>
<pin name="2" x="2.54" y="-5.08" visible="pad" length="middle" direction="pas"/>
<pin name="3" x="2.54" y="-10.16" visible="pad" length="middle" direction="pas"/>
<pin name="4" x="25.4" y="-10.16" visible="pad" length="middle" direction="pas" rot="R180"/>
<pin name="5" x="25.4" y="-5.08" visible="pad" length="middle" direction="pas" rot="R180"/>
<pin name="6" x="25.4" y="0" visible="pad" length="middle" direction="pas" rot="R180"/>
<wire x1="7.62" y1="2.54" x2="7.62" y2="0" width="0.1524" layer="94"/>
<wire x1="7.62" y1="0" x2="7.62" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-5.08" x2="7.62" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-10.16" x2="7.62" y2="-12.7" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-12.7" x2="20.32" y2="-12.7" width="0.1524" layer="94"/>
<wire x1="20.32" y1="-12.7" x2="20.32" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="20.32" y1="-10.16" x2="20.32" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="20.32" y1="-5.08" x2="20.32" y2="0" width="0.1524" layer="94"/>
<wire x1="20.32" y1="0" x2="20.32" y2="2.54" width="0.1524" layer="94"/>
<wire x1="20.32" y1="2.54" x2="7.62" y2="2.54" width="0.1524" layer="94"/>
<wire x1="7.62" y1="0" x2="9.525" y2="0" width="0.1524" layer="94"/>
<wire x1="9.525" y1="0" x2="12.065" y2="0" width="0.1524" layer="94"/>
<wire x1="12.065" y1="0" x2="13.97" y2="0" width="0.1524" layer="94"/>
<wire x1="13.97" y1="0" x2="13.97" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="13.335" y1="-1.905" x2="13.97" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="13.97" y1="-1.905" x2="14.605" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="15.24" y1="-1.905" x2="15.875" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="15.875" y1="-1.905" x2="16.51" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="12.7" y1="-1.905" x2="12.065" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="12.065" y1="-1.905" x2="11.43" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="12.065" y1="0" x2="12.065" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="9.525" y1="0" x2="9.525" y2="1.27" width="0.1524" layer="94"/>
<wire x1="15.875" y1="0" x2="17.78" y2="0" width="0.1524" layer="94"/>
<wire x1="17.78" y1="0" x2="20.32" y2="0" width="0.1524" layer="94"/>
<wire x1="15.875" y1="0" x2="15.875" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="13.97" y1="-1.905" x2="13.335" y2="-0.635" width="0.1524" layer="94"/>
<wire x1="13.335" y1="-0.635" x2="14.605" y2="-0.635" width="0.1524" layer="94"/>
<wire x1="13.97" y1="-1.905" x2="14.605" y2="-0.635" width="0.1524" layer="94"/>
<wire x1="9.525" y1="1.27" x2="14.605" y2="1.27" width="0.1524" layer="94"/>
<wire x1="14.605" y1="1.27" x2="17.78" y2="1.27" width="0.1524" layer="94"/>
<wire x1="17.78" y1="1.27" x2="17.78" y2="0" width="0.1524" layer="94"/>
<wire x1="13.335" y1="1.905" x2="13.335" y2="0.635" width="0.1524" layer="94"/>
<wire x1="13.335" y1="1.905" x2="14.605" y2="1.27" width="0.1524" layer="94"/>
<wire x1="14.605" y1="1.27" x2="13.335" y2="0.635" width="0.1524" layer="94"/>
<wire x1="14.605" y1="1.905" x2="14.605" y2="0.635" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-5.08" x2="11.43" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="11.43" y1="-5.08" x2="11.43" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="11.43" y1="-2.54" x2="16.51" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="20.32" y1="-10.16" x2="18.415" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="18.415" y1="-10.16" x2="15.875" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="15.875" y1="-10.16" x2="13.97" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="13.97" y1="-10.16" x2="13.97" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="14.605" y1="-8.255" x2="13.97" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="13.97" y1="-8.255" x2="13.335" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="12.7" y1="-8.255" x2="12.065" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="12.065" y1="-8.255" x2="11.43" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="15.24" y1="-8.255" x2="15.875" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="15.875" y1="-8.255" x2="16.51" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="15.875" y1="-10.16" x2="15.875" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="18.415" y1="-10.16" x2="18.415" y2="-11.43" width="0.1524" layer="94"/>
<wire x1="12.065" y1="-10.16" x2="10.16" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-10.16" x2="7.62" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="12.065" y1="-10.16" x2="12.065" y2="-8.255" width="0.1524" layer="94"/>
<wire x1="13.97" y1="-8.255" x2="14.605" y2="-9.525" width="0.1524" layer="94"/>
<wire x1="14.605" y1="-9.525" x2="13.335" y2="-9.525" width="0.1524" layer="94"/>
<wire x1="13.97" y1="-8.255" x2="13.335" y2="-9.525" width="0.1524" layer="94"/>
<wire x1="18.415" y1="-11.43" x2="13.335" y2="-11.43" width="0.1524" layer="94"/>
<wire x1="13.335" y1="-11.43" x2="10.16" y2="-11.43" width="0.1524" layer="94"/>
<wire x1="10.16" y1="-11.43" x2="10.16" y2="-10.16" width="0.1524" layer="94"/>
<wire x1="14.605" y1="-12.065" x2="14.605" y2="-10.795" width="0.1524" layer="94"/>
<wire x1="14.605" y1="-12.065" x2="13.335" y2="-11.43" width="0.1524" layer="94"/>
<wire x1="13.335" y1="-11.43" x2="14.605" y2="-10.795" width="0.1524" layer="94"/>
<wire x1="13.335" y1="-12.065" x2="13.335" y2="-10.795" width="0.1524" layer="94"/>
<wire x1="20.32" y1="-5.08" x2="16.51" y2="-5.08" width="0.1524" layer="94"/>
<wire x1="16.51" y1="-5.08" x2="16.51" y2="-7.62" width="0.1524" layer="94"/>
<wire x1="16.51" y1="-7.62" x2="11.43" y2="-7.62" width="0.1524" layer="94"/>
<polygon width="0.0254" layer="94">
<vertex x="13.335" y="-0.635"/>
<vertex x="13.97" y="-1.905"/>
<vertex x="14.605" y="-0.635"/>
</polygon>
<polygon width="0.0254" layer="94">
<vertex x="13.335" y="1.905"/>
<vertex x="13.335" y="0.635"/>
<vertex x="14.605" y="1.27"/>
</polygon>
<polygon width="0.0254" layer="94">
<vertex x="14.605" y="-9.525"/>
<vertex x="13.97" y="-8.255"/>
<vertex x="13.335" y="-9.525"/>
</polygon>
<polygon width="0.0254" layer="94">
<vertex x="14.605" y="-12.065"/>
<vertex x="14.605" y="-10.795"/>
<vertex x="13.335" y="-11.43"/>
</polygon>
<text x="10.5156" y="7.2136" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="9.8806" y="4.6736" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="MCM3400A-TP" prefix="Q">
<gates>
<gate name="A" symbol="DFN2020-6L" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DFN2020-6L_MCE">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="MCM3400A-TP" constant="no"/>
<attribute name="MFR_NAME" value="MCC" constant="no"/>
<attribute name="TYPE" value="Transistors" constant="no"/>
</technology>
</technologies>
</device>
<device name="DFN2020-6L_MCE-M" package="DFN2020-6L_MCE-M">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="MCM3400A-TP" constant="no"/>
<attribute name="MFR_NAME" value="MCC" constant="no"/>
<attribute name="TYPE" value="Transistors" constant="no"/>
</technology>
</technologies>
</device>
<device name="DFN2020-6L_MCE-L" package="DFN2020-6L_MCE-L">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
<connect gate="A" pin="5" pad="5"/>
<connect gate="A" pin="6" pad="6"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="MCM3400A-TP" constant="no"/>
<attribute name="MFR_NAME" value="MCC" constant="no"/>
<attribute name="TYPE" value="Transistors" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="LIS2DE12TR">
<packages>
<package name="LGA-12_STM">
<smd name="1" x="-0.8509" y="0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="2" x="-0.8509" y="0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="3" x="-0.8509" y="-0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="4" x="-0.8509" y="-0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="5" x="-0.25" y="-0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="6" x="0.25" y="-0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="7" x="0.8509" y="-0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="8" x="0.8509" y="-0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="9" x="0.8509" y="0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="10" x="0.8509" y="0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="11" x="0.25" y="0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="12" x="-0.25" y="0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<wire x1="-1.2192" y1="-1.2192" x2="-1.0414" y2="-1.2192" width="0.1524" layer="21"/>
<wire x1="1.2192" y1="1.2192" x2="1.0414" y2="1.2192" width="0.1524" layer="21"/>
<wire x1="-1.0414" y1="1.2192" x2="-1.2192" y2="1.2192" width="0.1524" layer="21"/>
<wire x1="1.0414" y1="-1.2192" x2="1.2192" y2="-1.2192" width="0.1524" layer="21"/>
<polygon width="0.0254" layer="21">
<vertex x="1.4986" y="0.9405"/>
<vertex x="1.4986" y="0.5595"/>
<vertex x="1.2446" y="0.5595"/>
<vertex x="1.2446" y="0.9405"/>
</polygon>
<text x="-2.2098" y="0.3556" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<wire x1="0.8636" y1="0.762" x2="0.9906" y2="0.762" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="0.762" x2="3.5306" y2="0.762" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.9116" y2="0.762" width="0.1524" layer="47"/>
<wire x1="0.8636" y1="0.254" x2="3.5306" y2="0.254" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.9116" y2="0.254" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.5306" y2="2.032" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.5306" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.4036" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.6576" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.4036" y1="1.016" x2="3.6576" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.4036" y2="0" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.6576" y2="0" width="0.1524" layer="47"/>
<wire x1="3.4036" y1="0" x2="3.6576" y2="0" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="0.762" x2="0.7112" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.7112" y2="3.9116" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="0.762" x2="0.9906" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="-0.5588" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="2.2606" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.4572" y2="3.6576" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.4572" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.4572" y1="3.6576" x2="0.4572" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="1.2446" y2="3.6576" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="1.2446" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="1.2446" y1="3.6576" x2="1.2446" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="0.762" x2="-0.9906" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-0.9906" y2="5.8166" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="0.9906" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="0.9906" y2="5.8166" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-2.2606" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="2.2606" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-1.2446" y2="5.5626" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.2446" y1="5.5626" x2="-1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="1.2446" y2="5.5626" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.2446" y1="5.5626" x2="1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="0.254" y1="0.9906" x2="5.4356" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.8166" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="0.254" y1="-0.9906" x2="5.4356" y2="-0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.8166" y2="-0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.4356" y2="2.2606" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.4356" y2="-2.2606" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.3086" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.5626" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.3086" y1="1.2446" x2="5.5626" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.3086" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.5626" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="5.3086" y1="-1.2446" x2="5.5626" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="1.0668" x2="-1.0668" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="1.0668" x2="-4.1656" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.5466" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-1.0668" x2="-4.1656" y2="-1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.5466" y2="-1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.1656" y2="2.3368" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.1656" y2="-2.3368" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.2926" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.0386" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.2926" y1="1.3208" x2="-4.0386" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.2926" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.0386" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-4.2926" y1="-1.3208" x2="-4.0386" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="1.0668" x2="-1.0668" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.0668" y2="-4.5466" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="1.0668" x2="1.0668" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.0668" y2="-4.5466" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-2.3368" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="2.3368" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.3208" y2="-4.0386" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="-1.3208" y1="-4.0386" x2="-1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.3208" y2="-4.0386" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="-4.0386" x2="1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<text x="-15.2146" y="-8.8646" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX10Y11D0T</text>
<text x="-14.2494" y="-13.4366" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: RX14Y15D0T</text>
<text x="-14.8082" y="-14.9606" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="4.0386" y="0.1778" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.5mm</text>
<text x="-3.2004" y="4.0386" size="0.635" layer="47" ratio="4" rot="SR0">0.011in/0.279mm</text>
<text x="-4.0386" y="5.9436" size="0.635" layer="47" ratio="4" rot="SR0">0.078in/1.981mm</text>
<text x="5.9436" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.078in/1.981mm</text>
<text x="-12.7508" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.085in/2.159mm</text>
<text x="-4.0386" y="-5.3086" size="0.635" layer="47" ratio="4" rot="SR0">0.085in/2.159mm</text>
<wire x1="-1.0668" y1="-0.2032" x2="-0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.127" x2="0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="0.9906" x2="0.2032" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="0.127" y1="1.0668" x2="0.381" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="0.381" y1="1.0668" x2="0.381" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0.9906" x2="0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="0.9906" x2="0.127" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="1.0668" x2="-0.127" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="1.0668" x2="-0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="0.9906" x2="-0.381" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="0.9906" x2="-0.381" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.635" x2="-1.0668" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.889" x2="-0.9906" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.889" x2="-0.9906" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.635" x2="-1.0668" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.127" x2="-1.0668" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.381" x2="-0.9906" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.381" x2="-0.9906" y2="0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.127" x2="-1.0668" y2="0.127" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.381" x2="-1.0668" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.127" x2="-0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.127" x2="-0.9906" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.381" x2="-1.0668" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.889" x2="-1.0668" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.635" x2="-0.9906" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.635" x2="-0.9906" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.889" x2="-1.0668" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="-1.0668" x2="-0.381" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="-1.0668" x2="-0.381" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="-0.9906" x2="-0.127" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="-0.9906" x2="-0.127" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-1.0668" x2="0.127" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="0.127" y1="-1.0668" x2="0.127" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="-0.9906" x2="0.381" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-0.9906" x2="0.381" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.635" x2="1.0668" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.889" x2="0.9906" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.889" x2="0.9906" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.635" x2="1.0668" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.127" x2="1.0668" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.381" x2="0.9906" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.381" x2="0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.127" x2="1.0668" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.381" x2="1.0668" y2="0.127" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.127" x2="0.9906" y2="0.127" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.127" x2="0.9906" y2="0.381" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.381" x2="1.0668" y2="0.381" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.889" x2="1.0668" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.635" x2="0.9906" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.635" x2="0.9906" y2="0.889" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.889" x2="1.0668" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-1.0668" x2="1.0668" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-1.0668" x2="1.0668" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="1.0668" x2="-1.0668" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="1.0668" x2="-1.0668" y2="-1.0668" width="0.1524" layer="51"/>
<text x="-1.2954" y="0.3556" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
<package name="LGA-12_STM-M">
<smd name="1" x="-0.8509" y="0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="2" x="-0.8509" y="0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="3" x="-0.8509" y="-0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="4" x="-0.8509" y="-0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="5" x="-0.25" y="-0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="6" x="0.25" y="-0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="7" x="0.8509" y="-0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="8" x="0.8509" y="-0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="9" x="0.8509" y="0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="10" x="0.8509" y="0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="11" x="0.25" y="0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="12" x="-0.25" y="0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<polygon width="0.0254" layer="21">
<vertex x="1.5494" y="0.9405"/>
<vertex x="1.5494" y="0.5595"/>
<vertex x="1.2954" y="0.5595"/>
<vertex x="1.2954" y="0.9405"/>
</polygon>
<text x="-2.2606" y="0.3556" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<wire x1="0.8636" y1="0.762" x2="0.9906" y2="0.762" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="0.762" x2="3.5306" y2="0.762" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.9116" y2="0.762" width="0.1524" layer="47"/>
<wire x1="0.8636" y1="0.254" x2="3.5306" y2="0.254" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.9116" y2="0.254" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.5306" y2="2.032" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.5306" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.4036" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.6576" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.4036" y1="1.016" x2="3.6576" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.4036" y2="0" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.6576" y2="0" width="0.1524" layer="47"/>
<wire x1="3.4036" y1="0" x2="3.6576" y2="0" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="0.762" x2="0.7112" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.7112" y2="3.9116" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="0.762" x2="0.9906" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="-0.5588" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="2.2606" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.4572" y2="3.6576" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.4572" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.4572" y1="3.6576" x2="0.4572" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="1.2446" y2="3.6576" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="1.2446" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="1.2446" y1="3.6576" x2="1.2446" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="0.762" x2="-0.9906" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-0.9906" y2="5.8166" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="0.9906" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="0.9906" y2="5.8166" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-2.2606" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="2.2606" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-1.2446" y2="5.5626" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.2446" y1="5.5626" x2="-1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="1.2446" y2="5.5626" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.2446" y1="5.5626" x2="1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="0.254" y1="0.9906" x2="5.4356" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.8166" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="0.254" y1="-0.9906" x2="5.4356" y2="-0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.8166" y2="-0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.4356" y2="2.2606" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.4356" y2="-2.2606" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.3086" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.5626" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.3086" y1="1.2446" x2="5.5626" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.3086" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.5626" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="5.3086" y1="-1.2446" x2="5.5626" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="1.0668" x2="-1.0668" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="1.0668" x2="-4.1656" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.5466" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-1.0668" x2="-4.1656" y2="-1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.5466" y2="-1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.1656" y2="2.3368" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.1656" y2="-2.3368" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.2926" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.0386" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.2926" y1="1.3208" x2="-4.0386" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.2926" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.0386" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-4.2926" y1="-1.3208" x2="-4.0386" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="1.0668" x2="-1.0668" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.0668" y2="-4.5466" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="1.0668" x2="1.0668" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.0668" y2="-4.5466" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-2.3368" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="2.3368" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.3208" y2="-4.0386" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="-1.3208" y1="-4.0386" x2="-1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.3208" y2="-4.0386" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="-4.0386" x2="1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<text x="-15.2146" y="-8.9154" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX12Y15D0T</text>
<text x="-14.2494" y="-13.4874" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: RX14Y15D0T</text>
<text x="-14.8082" y="-15.0114" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="4.0386" y="0.1778" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.5mm</text>
<text x="-3.2004" y="4.0386" size="0.635" layer="47" ratio="4" rot="SR0">0.011in/0.279mm</text>
<text x="-4.0386" y="5.9436" size="0.635" layer="47" ratio="4" rot="SR0">0.078in/1.981mm</text>
<text x="5.9436" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.078in/1.981mm</text>
<text x="-12.7508" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.085in/2.159mm</text>
<text x="-4.0386" y="-5.3086" size="0.635" layer="47" ratio="4" rot="SR0">0.085in/2.159mm</text>
<wire x1="-1.0668" y1="-0.2032" x2="-0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.127" x2="0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="0.9906" x2="0.2032" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="0.127" y1="1.0668" x2="0.381" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="0.381" y1="1.0668" x2="0.381" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0.9906" x2="0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="0.9906" x2="0.127" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="1.0668" x2="-0.127" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="1.0668" x2="-0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="0.9906" x2="-0.381" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="0.9906" x2="-0.381" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.635" x2="-1.0668" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.889" x2="-0.9906" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.889" x2="-0.9906" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.635" x2="-1.0668" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.127" x2="-1.0668" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.381" x2="-0.9906" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.381" x2="-0.9906" y2="0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.127" x2="-1.0668" y2="0.127" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.381" x2="-1.0668" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.127" x2="-0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.127" x2="-0.9906" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.381" x2="-1.0668" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.889" x2="-1.0668" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.635" x2="-0.9906" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.635" x2="-0.9906" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.889" x2="-1.0668" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="-1.0668" x2="-0.381" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="-1.0668" x2="-0.381" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="-0.9906" x2="-0.127" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="-0.9906" x2="-0.127" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-1.0668" x2="0.127" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="0.127" y1="-1.0668" x2="0.127" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="-0.9906" x2="0.381" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-0.9906" x2="0.381" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.635" x2="1.0668" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.889" x2="0.9906" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.889" x2="0.9906" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.635" x2="1.0668" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.127" x2="1.0668" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.381" x2="0.9906" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.381" x2="0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.127" x2="1.0668" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.381" x2="1.0668" y2="0.127" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.127" x2="0.9906" y2="0.127" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.127" x2="0.9906" y2="0.381" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.381" x2="1.0668" y2="0.381" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.889" x2="1.0668" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.635" x2="0.9906" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.635" x2="0.9906" y2="0.889" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.889" x2="1.0668" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-1.0668" x2="1.0668" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-1.0668" x2="1.0668" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="1.0668" x2="-1.0668" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="1.0668" x2="-1.0668" y2="-1.0668" width="0.1524" layer="51"/>
<text x="-1.2446" y="0.3556" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="25" ratio="6" rot="SR0">&gt;Name</text>
</package>
<package name="LGA-12_STM-L">
<smd name="1" x="-0.8509" y="0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="2" x="-0.8509" y="0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="3" x="-0.8509" y="-0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="4" x="-0.8509" y="-0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="5" x="-0.25" y="-0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="6" x="0.25" y="-0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="7" x="0.8509" y="-0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="8" x="0.8509" y="-0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="9" x="0.8509" y="0.25" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="10" x="0.8509" y="0.75" dx="0.3556" dy="0.381" layer="1" rot="R270"/>
<smd name="11" x="0.25" y="0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<smd name="12" x="-0.25" y="0.8509" dx="0.3556" dy="0.381" layer="1" rot="R180"/>
<polygon width="0.0254" layer="21">
<vertex x="1.4478" y="0.9405"/>
<vertex x="1.4478" y="0.5595"/>
<vertex x="1.1938" y="0.5595"/>
<vertex x="1.1938" y="0.9405"/>
</polygon>
<text x="-2.159" y="0.3556" size="1.27" layer="21" ratio="6" rot="SR0">*</text>
<wire x1="0.8636" y1="0.762" x2="0.9906" y2="0.762" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="0.762" x2="3.5306" y2="0.762" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.9116" y2="0.762" width="0.1524" layer="47"/>
<wire x1="0.8636" y1="0.254" x2="3.5306" y2="0.254" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.9116" y2="0.254" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.5306" y2="2.032" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.5306" y2="-1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.4036" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.762" x2="3.6576" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.4036" y1="1.016" x2="3.6576" y2="1.016" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.4036" y2="0" width="0.1524" layer="47"/>
<wire x1="3.5306" y1="0.254" x2="3.6576" y2="0" width="0.1524" layer="47"/>
<wire x1="3.4036" y1="0" x2="3.6576" y2="0" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="0.762" x2="0.7112" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.7112" y2="3.9116" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="0.762" x2="0.9906" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="-0.5588" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="2.2606" y2="3.5306" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.4572" y2="3.6576" width="0.1524" layer="47"/>
<wire x1="0.7112" y1="3.5306" x2="0.4572" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.4572" y1="3.6576" x2="0.4572" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="1.2446" y2="3.6576" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="1.2446" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="1.2446" y1="3.6576" x2="1.2446" y2="3.4036" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="0.762" x2="-0.9906" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-0.9906" y2="5.8166" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="3.5306" x2="0.9906" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="0.9906" y2="5.8166" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-2.2606" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="2.2606" y2="5.4356" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-1.2446" y2="5.5626" width="0.1524" layer="47"/>
<wire x1="-0.9906" y1="5.4356" x2="-1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="-1.2446" y1="5.5626" x2="-1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="1.2446" y2="5.5626" width="0.1524" layer="47"/>
<wire x1="0.9906" y1="5.4356" x2="1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="1.2446" y1="5.5626" x2="1.2446" y2="5.3086" width="0.1524" layer="47"/>
<wire x1="0.254" y1="0.9906" x2="5.4356" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.8166" y2="0.9906" width="0.1524" layer="47"/>
<wire x1="0.254" y1="-0.9906" x2="5.4356" y2="-0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.8166" y2="-0.9906" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.4356" y2="2.2606" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.4356" y2="-2.2606" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.3086" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="0.9906" x2="5.5626" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.3086" y1="1.2446" x2="5.5626" y2="1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.3086" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="5.4356" y1="-0.9906" x2="5.5626" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="5.3086" y1="-1.2446" x2="5.5626" y2="-1.2446" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="1.0668" x2="-1.0668" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="1.0668" x2="-4.1656" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.5466" y2="1.0668" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-1.0668" x2="-4.1656" y2="-1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.5466" y2="-1.0668" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.1656" y2="2.3368" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.1656" y2="-2.3368" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.2926" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="1.0668" x2="-4.0386" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.2926" y1="1.3208" x2="-4.0386" y2="1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.2926" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-4.1656" y1="-1.0668" x2="-4.0386" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-4.2926" y1="-1.3208" x2="-4.0386" y2="-1.3208" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="1.0668" x2="-1.0668" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.0668" y2="-4.5466" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="1.0668" x2="1.0668" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.0668" y2="-4.5466" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-2.3368" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="2.3368" y2="-4.1656" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.3208" y2="-4.0386" width="0.1524" layer="47"/>
<wire x1="-1.0668" y1="-4.1656" x2="-1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="-1.3208" y1="-4.0386" x2="-1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.3208" y2="-4.0386" width="0.1524" layer="47"/>
<wire x1="1.0668" y1="-4.1656" x2="1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<wire x1="1.3208" y1="-4.0386" x2="1.3208" y2="-4.2926" width="0.1524" layer="47"/>
<text x="-14.0462" y="-8.8138" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX8Y7D0T</text>
<text x="-14.2494" y="-13.3858" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: RX14Y15D0T</text>
<text x="-14.8082" y="-14.9098" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="4.0386" y="0.1778" size="0.635" layer="47" ratio="4" rot="SR0">0.02in/0.5mm</text>
<text x="-3.2004" y="4.0386" size="0.635" layer="47" ratio="4" rot="SR0">0.011in/0.279mm</text>
<text x="-4.0386" y="5.9436" size="0.635" layer="47" ratio="4" rot="SR0">0.078in/1.981mm</text>
<text x="5.9436" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.078in/1.981mm</text>
<text x="-12.7508" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.085in/2.159mm</text>
<text x="-4.0386" y="-5.3086" size="0.635" layer="47" ratio="4" rot="SR0">0.085in/2.159mm</text>
<wire x1="-1.0668" y1="-0.2032" x2="-0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.127" x2="0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="0.9906" x2="0.2032" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="0.127" y1="1.0668" x2="0.381" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="0.381" y1="1.0668" x2="0.381" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0.9906" x2="0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="0.9906" x2="0.127" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="1.0668" x2="-0.127" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="1.0668" x2="-0.127" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="0.9906" x2="-0.381" y2="0.9906" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="0.9906" x2="-0.381" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.635" x2="-1.0668" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.889" x2="-0.9906" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.889" x2="-0.9906" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.635" x2="-1.0668" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.127" x2="-1.0668" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="0.381" x2="-0.9906" y2="0.381" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.381" x2="-0.9906" y2="0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="0.127" x2="-1.0668" y2="0.127" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.381" x2="-1.0668" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.127" x2="-0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.127" x2="-0.9906" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.381" x2="-1.0668" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.889" x2="-1.0668" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-0.635" x2="-0.9906" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.635" x2="-0.9906" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-0.9906" y1="-0.889" x2="-1.0668" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="-1.0668" x2="-0.381" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="-1.0668" x2="-0.381" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="-0.381" y1="-0.9906" x2="-0.127" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="-0.127" y1="-0.9906" x2="-0.127" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-1.0668" x2="0.127" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="0.127" y1="-1.0668" x2="0.127" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="0.127" y1="-0.9906" x2="0.381" y2="-0.9906" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-0.9906" x2="0.381" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.635" x2="1.0668" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.889" x2="0.9906" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.889" x2="0.9906" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.635" x2="1.0668" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.127" x2="1.0668" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-0.381" x2="0.9906" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.381" x2="0.9906" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="-0.127" x2="1.0668" y2="-0.127" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.381" x2="1.0668" y2="0.127" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.127" x2="0.9906" y2="0.127" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.127" x2="0.9906" y2="0.381" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.381" x2="1.0668" y2="0.381" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.889" x2="1.0668" y2="0.635" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="0.635" x2="0.9906" y2="0.635" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.635" x2="0.9906" y2="0.889" width="0.1524" layer="51"/>
<wire x1="0.9906" y1="0.889" x2="1.0668" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="-1.0668" x2="1.0668" y2="-1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="-1.0668" x2="1.0668" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="1.0668" y1="1.0668" x2="-1.0668" y2="1.0668" width="0.1524" layer="51"/>
<wire x1="-1.0668" y1="1.0668" x2="-1.0668" y2="-1.0668" width="0.1524" layer="51"/>
<text x="-1.3462" y="0.3556" size="1.27" layer="51" ratio="6" rot="SR0">*</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
<text x="-3.2766" y="-0.635" size="1.27" layer="25" ratio="6" rot="SR0">&gt;Name</text>
</package>
</packages>
<symbols>
<symbol name="LIS2DE12">
<pin name="SCL/SPC" x="2.54" y="0" length="middle" direction="in"/>
<pin name="CS" x="2.54" y="-2.54" length="middle" direction="in"/>
<pin name="SDO/SA0" x="2.54" y="-5.08" length="middle"/>
<pin name="SDA/SDI/SDO" x="2.54" y="-7.62" length="middle"/>
<pin name="RES" x="2.54" y="-10.16" length="middle" direction="pas"/>
<pin name="GND_2" x="2.54" y="-12.7" length="middle" direction="pas"/>
<pin name="GND_3" x="58.42" y="-12.7" length="middle" direction="pas" rot="R180"/>
<pin name="GND" x="58.42" y="-10.16" length="middle" direction="pas" rot="R180"/>
<pin name="VDD" x="58.42" y="-7.62" length="middle" direction="pwr" rot="R180"/>
<pin name="VDD_IO" x="58.42" y="-5.08" length="middle" direction="pwr" rot="R180"/>
<pin name="INT2" x="58.42" y="-2.54" length="middle" rot="R180"/>
<pin name="INT1" x="58.42" y="0" length="middle" rot="R180"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-17.78" width="0.1524" layer="94"/>
<wire x1="7.62" y1="-17.78" x2="53.34" y2="-17.78" width="0.1524" layer="94"/>
<wire x1="53.34" y1="-17.78" x2="53.34" y2="5.08" width="0.1524" layer="94"/>
<wire x1="53.34" y1="5.08" x2="7.62" y2="5.08" width="0.1524" layer="94"/>
<text x="25.7556" y="9.1186" size="2.0828" layer="95" ratio="6" rot="SR0">&gt;Name</text>
<text x="25.1206" y="6.5786" size="2.0828" layer="96" ratio="6" rot="SR0">&gt;Value</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="LIS2DE12TR" prefix="U">
<gates>
<gate name="A" symbol="LIS2DE12" x="0" y="0"/>
</gates>
<devices>
<device name="" package="LGA-12_STM">
<connects>
<connect gate="A" pin="CS" pad="2"/>
<connect gate="A" pin="GND" pad="8"/>
<connect gate="A" pin="GND_2" pad="6"/>
<connect gate="A" pin="GND_3" pad="7"/>
<connect gate="A" pin="INT1" pad="12"/>
<connect gate="A" pin="INT2" pad="11"/>
<connect gate="A" pin="RES" pad="5"/>
<connect gate="A" pin="SCL/SPC" pad="1"/>
<connect gate="A" pin="SDA/SDI/SDO" pad="4"/>
<connect gate="A" pin="SDO/SA0" pad="3"/>
<connect gate="A" pin="VDD" pad="9"/>
<connect gate="A" pin="VDD_IO" pad="10"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="LIS2DE12TR" constant="no"/>
<attribute name="MFR_NAME" value="STMicroelectronics" constant="no"/>
</technology>
</technologies>
</device>
<device name="LGA-12_STM-M" package="LGA-12_STM-M">
<connects>
<connect gate="A" pin="CS" pad="2"/>
<connect gate="A" pin="GND" pad="8"/>
<connect gate="A" pin="GND_2" pad="6"/>
<connect gate="A" pin="GND_3" pad="7"/>
<connect gate="A" pin="INT1" pad="12"/>
<connect gate="A" pin="INT2" pad="11"/>
<connect gate="A" pin="RES" pad="5"/>
<connect gate="A" pin="SCL/SPC" pad="1"/>
<connect gate="A" pin="SDA/SDI/SDO" pad="4"/>
<connect gate="A" pin="SDO/SA0" pad="3"/>
<connect gate="A" pin="VDD" pad="9"/>
<connect gate="A" pin="VDD_IO" pad="10"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="LIS2DE12TR" constant="no"/>
<attribute name="MFR_NAME" value="STMicroelectronics" constant="no"/>
</technology>
</technologies>
</device>
<device name="LGA-12_STM-L" package="LGA-12_STM-L">
<connects>
<connect gate="A" pin="CS" pad="2"/>
<connect gate="A" pin="GND" pad="8"/>
<connect gate="A" pin="GND_2" pad="6"/>
<connect gate="A" pin="GND_3" pad="7"/>
<connect gate="A" pin="INT1" pad="12"/>
<connect gate="A" pin="INT2" pad="11"/>
<connect gate="A" pin="RES" pad="5"/>
<connect gate="A" pin="SCL/SPC" pad="1"/>
<connect gate="A" pin="SDA/SDI/SDO" pad="4"/>
<connect gate="A" pin="SDO/SA0" pad="3"/>
<connect gate="A" pin="VDD" pad="9"/>
<connect gate="A" pin="VDD_IO" pad="10"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="LIS2DE12TR" constant="no"/>
<attribute name="MFR_NAME" value="STMicroelectronics" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="74438357010">
<packages>
<package name="IND_74438357010_WRE">
<smd name="1" x="-1.4224" y="0" dx="1.4478" dy="4.2926" layer="1"/>
<smd name="2" x="1.4224" y="0" dx="1.4478" dy="4.2926" layer="1"/>
<wire x1="-2.1336" y1="0" x2="-2.1336" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-2.1336" y2="6.731" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="0" x2="2.1336" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="2.1336" x2="2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="2.1336" y2="6.731" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-1.8796" y2="6.477" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-1.8796" y1="6.477" x2="-1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="1.8796" y2="6.477" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="1.8796" y1="6.477" x2="1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="0" x2="-0.7112" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.7112" y2="2.921" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-3.4036" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="0.5588" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.3876" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.3876" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="2.667" x2="-2.3876" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.4572" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.4572" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.4572" y1="2.667" x2="-0.4572" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.4224" y1="0" x2="-3.9624" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-4.3434" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.4224" y1="-0.635" x2="-3.9624" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-4.3434" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-3.9624" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-3.9624" y2="-1.905" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-4.0894" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-3.8354" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.0894" y1="0.254" x2="-3.8354" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-4.0894" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-3.8354" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="-4.0894" y1="-0.889" x2="-3.8354" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="2.1336" x2="3.9624" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="4.3434" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-2.1336" x2="3.9624" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="4.3434" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="3.9624" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="3.8354" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="4.0894" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="1.8796" x2="4.0894" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="3.8354" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="4.0894" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.8796" x2="4.0894" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-2.1336" x2="-2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-2.1336" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-2.1336" x2="2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="2.1336" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-1.8796" y2="-4.572" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="-1.8796" y1="-4.572" x2="-1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="1.8796" y2="-4.572" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="1.8796" y1="-4.572" x2="1.8796" y2="-4.826" width="0.1524" layer="47"/>
<text x="-15.0114" y="-10.0076" size="1.27" layer="47" ratio="6" rot="SR0">Pin 1 Padstyle: RX57Y169D0T</text>
<text x="-17.5006" y="-11.5316" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX57Y169D0TSM2</text>
<text x="-14.8082" y="-16.1036" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.6276" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-4.0386" y="6.858" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<text x="-5.461" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.057in/1.448mm</text>
<text x="-12.5476" y="-0.635" size="0.635" layer="47" ratio="4" rot="SR0">0.025in/0.635mm</text>
<text x="4.4704" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<text x="-4.0386" y="-5.842" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<wire x1="-2.1336" y1="-2.1336" x2="2.1336" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="-2.1336" x2="2.1336" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="2.1336" x2="-2.1336" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="-2.1336" y1="2.1336" x2="-2.1336" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="0.0762" y1="0" x2="-0.0762" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-0.0762" y1="0" x2="0.0762" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-0.381" y1="-2.286" x2="0.381" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="0.381" y1="2.286" x2="-0.381" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-3.2258" y1="0" x2="-3.3782" y2="0" width="0.1524" layer="21" curve="-180"/>
<wire x1="-3.3782" y1="0" x2="-3.2258" y2="0" width="0.1524" layer="21" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
<package name="IND_74438357010_WRE-M">
<smd name="1" x="-1.4224" y="0" dx="1.5494" dy="4.3434" layer="1"/>
<smd name="2" x="1.4224" y="0" dx="1.5494" dy="4.3434" layer="1"/>
<wire x1="-2.1336" y1="0" x2="-2.1336" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-2.1336" y2="6.731" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="0" x2="2.1336" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="2.1336" x2="2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="2.1336" y2="6.731" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-1.8796" y2="6.477" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-1.8796" y1="6.477" x2="-1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="1.8796" y2="6.477" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="1.8796" y1="6.477" x2="1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="0" x2="-0.7112" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.7112" y2="2.921" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-3.4036" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="0.5588" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.3876" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.3876" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="2.667" x2="-2.3876" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.4572" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.4572" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.4572" y1="2.667" x2="-0.4572" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.4224" y1="0" x2="-3.9624" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-4.3434" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.4224" y1="-0.635" x2="-3.9624" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-4.3434" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-3.9624" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-3.9624" y2="-1.905" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-4.0894" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-3.8354" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.0894" y1="0.254" x2="-3.8354" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-4.0894" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-3.8354" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="-4.0894" y1="-0.889" x2="-3.8354" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="2.1336" x2="3.9624" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="4.3434" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-2.1336" x2="3.9624" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="4.3434" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="3.9624" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="3.8354" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="4.0894" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="1.8796" x2="4.0894" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="3.8354" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="4.0894" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.8796" x2="4.0894" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-2.1336" x2="-2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-2.1336" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-2.1336" x2="2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="2.1336" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-1.8796" y2="-4.572" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="-1.8796" y1="-4.572" x2="-1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="1.8796" y2="-4.572" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="1.8796" y1="-4.572" x2="1.8796" y2="-4.826" width="0.1524" layer="47"/>
<text x="-15.0114" y="-10.0076" size="1.27" layer="47" ratio="6" rot="SR0">Pin 1 Padstyle: RX61Y171D0T</text>
<text x="-17.5006" y="-11.5316" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX61Y171D0TSM2</text>
<text x="-14.8082" y="-16.1036" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.6276" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-4.0386" y="6.858" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<text x="-5.461" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.057in/1.448mm</text>
<text x="-12.5476" y="-0.635" size="0.635" layer="47" ratio="4" rot="SR0">0.025in/0.635mm</text>
<text x="4.4704" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<text x="-4.0386" y="-5.842" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<wire x1="-2.1336" y1="-2.1336" x2="2.1336" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="-2.1336" x2="2.1336" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="2.1336" x2="-2.1336" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="-2.1336" y1="2.1336" x2="-2.1336" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="0.0762" y1="0" x2="-0.0762" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-0.0762" y1="0" x2="0.0762" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-0.3302" y1="-2.286" x2="0.3302" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="2.286" x2="-0.3302" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-3.2766" y1="0" x2="-3.429" y2="0" width="0.1524" layer="21" curve="-180"/>
<wire x1="-3.429" y1="0" x2="-3.2766" y2="0" width="0.1524" layer="21" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
<package name="IND_74438357010_WRE-L">
<smd name="1" x="-1.4224" y="0" dx="1.3462" dy="4.2418" layer="1"/>
<smd name="2" x="1.4224" y="0" dx="1.3462" dy="4.2418" layer="1"/>
<wire x1="-2.1336" y1="0" x2="-2.1336" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-2.1336" y2="6.731" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="0" x2="2.1336" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="2.1336" x2="2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="2.1336" y2="6.731" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="2.1336" y2="6.35" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-1.8796" y2="6.477" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="6.35" x2="-1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-1.8796" y1="6.477" x2="-1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="1.8796" y2="6.477" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="6.35" x2="1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="1.8796" y1="6.477" x2="1.8796" y2="6.223" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="0" x2="-0.7112" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.7112" y2="2.921" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-3.4036" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="0.5588" y2="2.54" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.3876" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="2.54" x2="-2.3876" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-2.3876" y1="2.667" x2="-2.3876" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.4572" y2="2.667" width="0.1524" layer="47"/>
<wire x1="-0.7112" y1="2.54" x2="-0.4572" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-0.4572" y1="2.667" x2="-0.4572" y2="2.413" width="0.1524" layer="47"/>
<wire x1="-1.4224" y1="0" x2="-3.9624" y2="0" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-4.3434" y2="0" width="0.1524" layer="47"/>
<wire x1="-1.4224" y1="-0.635" x2="-3.9624" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-4.3434" y2="-0.635" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-3.9624" y2="1.27" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-3.9624" y2="-1.905" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-4.0894" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="0" x2="-3.8354" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-4.0894" y1="0.254" x2="-3.8354" y2="0.254" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-4.0894" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="-3.9624" y1="-0.635" x2="-3.8354" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="-4.0894" y1="-0.889" x2="-3.8354" y2="-0.889" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="2.1336" x2="3.9624" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="4.3434" y2="2.1336" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-2.1336" x2="3.9624" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="4.3434" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="3.9624" y2="-2.1336" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="3.8354" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="2.1336" x2="4.0894" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="1.8796" x2="4.0894" y2="1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="3.8354" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="3.9624" y1="-2.1336" x2="4.0894" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="3.8354" y1="-1.8796" x2="4.0894" y2="-1.8796" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-2.1336" x2="-2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-2.1336" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-2.1336" x2="2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="2.1336" y2="-5.08" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="2.1336" y2="-4.6736" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-1.8796" y2="-4.572" width="0.1524" layer="47"/>
<wire x1="-2.1336" y1="-4.6736" x2="-1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="-1.8796" y1="-4.572" x2="-1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="1.8796" y2="-4.572" width="0.1524" layer="47"/>
<wire x1="2.1336" y1="-4.6736" x2="1.8796" y2="-4.826" width="0.1524" layer="47"/>
<wire x1="1.8796" y1="-4.572" x2="1.8796" y2="-4.826" width="0.1524" layer="47"/>
<text x="-15.0114" y="-10.0076" size="1.27" layer="47" ratio="6" rot="SR0">Pin 1 Padstyle: RX53Y167D0T</text>
<text x="-17.5006" y="-11.5316" size="1.27" layer="47" ratio="6" rot="SR0">Default Padstyle: RX53Y167D0TSM2</text>
<text x="-14.8082" y="-16.1036" size="1.27" layer="47" ratio="6" rot="SR0">Alt 1 Padstyle: OX60Y90D30P</text>
<text x="-14.8082" y="-17.6276" size="1.27" layer="47" ratio="6" rot="SR0">Alt 2 Padstyle: OX90Y60D30P</text>
<text x="-4.0386" y="6.858" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<text x="-5.461" y="3.048" size="0.635" layer="47" ratio="4" rot="SR0">0.057in/1.448mm</text>
<text x="-12.5476" y="-0.635" size="0.635" layer="47" ratio="4" rot="SR0">0.025in/0.635mm</text>
<text x="4.4704" y="-0.3048" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<text x="-4.0386" y="-5.842" size="0.635" layer="47" ratio="4" rot="SR0">0.169in/4.293mm</text>
<wire x1="-2.1336" y1="-2.1336" x2="2.1336" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="-2.1336" x2="2.1336" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="2.1336" y1="2.1336" x2="-2.1336" y2="2.1336" width="0.1524" layer="51"/>
<wire x1="-2.1336" y1="2.1336" x2="-2.1336" y2="-2.1336" width="0.1524" layer="51"/>
<wire x1="0.0762" y1="0" x2="-0.0762" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-0.0762" y1="0" x2="0.0762" y2="0" width="0" layer="51" curve="-180"/>
<wire x1="-0.4572" y1="-2.286" x2="0.4572" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="0.4572" y1="2.286" x2="-0.4572" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.3274" y2="0" width="0.1524" layer="21" curve="-180"/>
<wire x1="-3.3274" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21" curve="-180"/>
<text x="-3.2766" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Name</text>
<text x="-1.7272" y="-0.635" size="1.27" layer="27" ratio="6" rot="SR0">&gt;Value</text>
</package>
</packages>
<symbols>
<symbol name="IND">
<pin name="1" x="15.24" y="0" visible="off" length="short" direction="pas" rot="R180"/>
<pin name="2" x="0" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<wire x1="5.08" y1="0" x2="5.08" y2="1.27" width="0.2032" layer="94"/>
<wire x1="7.62" y1="0" x2="7.62" y2="1.27" width="0.2032" layer="94"/>
<wire x1="12.7" y1="0" x2="12.7" y2="1.27" width="0.2032" layer="94"/>
<wire x1="2.54" y1="0" x2="2.54" y2="1.27" width="0.2032" layer="94"/>
<wire x1="10.16" y1="0" x2="10.16" y2="1.27" width="0.2032" layer="94"/>
<wire x1="5.08" y1="1.27" x2="7.62" y2="1.27" width="0.1524" layer="94" curve="-180"/>
<wire x1="2.54" y1="1.27" x2="5.08" y2="1.27" width="0.1524" layer="94" curve="-180"/>
<wire x1="7.62" y1="1.27" x2="10.16" y2="1.27" width="0.1524" layer="94" curve="-180"/>
<wire x1="10.16" y1="1.27" x2="12.7" y2="1.27" width="0.1524" layer="94" curve="-180"/>
<text x="-1.9812" y="-4.2672" size="3.4798" layer="96" ratio="10" rot="SR0">&gt;Value</text>
<text x="-0.9144" y="3.3528" size="3.4798" layer="95" ratio="10" rot="SR0">&gt;Name</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="74438357010" prefix="L">
<gates>
<gate name="A" symbol="IND" x="0" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="" package="IND_74438357010_WRE">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="74438357010" constant="no"/>
<attribute name="MFR_NAME" value="Wurth Electronics" constant="no"/>
</technology>
</technologies>
</device>
<device name="IND_74438357010_WRE-M" package="IND_74438357010_WRE-M">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="74438357010" constant="no"/>
<attribute name="MFR_NAME" value="Wurth Electronics" constant="no"/>
</technology>
</technologies>
</device>
<device name="IND_74438357010_WRE-L" package="IND_74438357010_WRE-L">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="COPYRIGHT" value="Copyright (C) 2024 Ultra Librarian. All rights reserved." constant="no"/>
<attribute name="MANUFACTURER_PART_NUMBER" value="74438357010" constant="no"/>
<attribute name="MFR_NAME" value="Wurth Electronics" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U1" library="ESP32-C3-WROOM-02-N4" deviceset="ESP32-C3-WROOM-02-N4" device=""/>
<part name="R1" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="10k"/>
<part name="C1" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="1uF"/>
<part name="C2" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="100nF"/>
<part name="C3" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="10uF"/>
<part name="R2" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="10k"/>
<part name="R3" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="10k"/>
<part name="U2" library="TPS61023" deviceset="TPS61023DRLR" device=""/>
<part name="IO9" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="1,6/0,8" device="" package3d_urn="urn:adsk.eagle:package:30830/1"/>
<part name="BAT+" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="1,6/0,8" device="" package3d_urn="urn:adsk.eagle:package:30830/1"/>
<part name="C4" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="10uF"/>
<part name="R4" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="1M"/>
<part name="R5" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="220k"/>
<part name="C5" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="22uF"/>
<part name="C6" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="22uF"/>
<part name="U3" library="VL53L1X" deviceset="VL53L1CXV0FY/1" device=""/>
<part name="U4" library="LP3985IM5X-2.8" deviceset="LP3985IM5X-2.8/NOPB" device=""/>
<part name="C7" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="1uF"/>
<part name="C8" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="1uF"/>
<part name="Q1" library="MCM3400A-TP" deviceset="MCM3400A-TP" device=""/>
<part name="R6" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="10k"/>
<part name="R7" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="10k"/>
<part name="R8" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="10k"/>
<part name="R9" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="10k"/>
<part name="R10" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="47k"/>
<part name="R11" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="R-US_" device="R0805" package3d_urn="urn:adsk.eagle:package:23553/2" value="1k"/>
<part name="U5" library="LIS2DE12TR" deviceset="LIS2DE12TR" device=""/>
<part name="C9" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="100nF"/>
<part name="C10" library="rcl" library_urn="urn:adsk.eagle:library:334" deviceset="C-US" device="C0805" package3d_urn="urn:adsk.eagle:package:23617/2" value="10uF"/>
<part name="3V3" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="1,6/0,8" device="" package3d_urn="urn:adsk.eagle:package:30830/1"/>
<part name="GND" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="1,6/0,8" device="" package3d_urn="urn:adsk.eagle:package:30830/1"/>
<part name="RX" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="1,6/0,8" device="" package3d_urn="urn:adsk.eagle:package:30830/1"/>
<part name="TX" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="1,6/0,8" device="" package3d_urn="urn:adsk.eagle:package:30830/1"/>
<part name="BAT-" library="wirepad" library_urn="urn:adsk.eagle:library:412" deviceset="1,6/0,8" device="" package3d_urn="urn:adsk.eagle:package:30830/1"/>
<part name="L1" library="74438357010" deviceset="74438357010" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U1" gate="G$1" x="38.1" y="40.64" smashed="yes">
<attribute name="NAME" x="25.4" y="61.722" size="1.778" layer="95"/>
<attribute name="VALUE" x="25.4" y="22.098" size="1.778" layer="96" align="top-left"/>
</instance>
<instance part="R1" gate="G$1" x="27.94" y="68.58" smashed="yes">
<attribute name="NAME" x="24.13" y="70.0786" size="1.778" layer="95"/>
<attribute name="VALUE" x="24.13" y="65.278" size="1.778" layer="96"/>
</instance>
<instance part="C1" gate="G$1" x="17.78" y="71.12" smashed="yes" rot="R180">
<attribute name="NAME" x="16.764" y="70.485" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="16.764" y="75.311" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C2" gate="G$1" x="38.1" y="71.12" smashed="yes" rot="R180">
<attribute name="NAME" x="37.084" y="70.485" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="37.084" y="75.311" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C3" gate="G$1" x="48.26" y="71.12" smashed="yes" rot="R180">
<attribute name="NAME" x="47.244" y="70.485" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="47.244" y="75.311" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R2" gate="G$1" x="10.16" y="45.72" smashed="yes">
<attribute name="NAME" x="6.35" y="47.2186" size="1.778" layer="95"/>
<attribute name="VALUE" x="6.35" y="42.418" size="1.778" layer="96"/>
</instance>
<instance part="R3" gate="G$1" x="60.96" y="48.26" smashed="yes">
<attribute name="NAME" x="57.15" y="49.7586" size="1.778" layer="95"/>
<attribute name="VALUE" x="62.23" y="50.038" size="1.778" layer="96"/>
</instance>
<instance part="U2" gate="A" x="106.68" y="38.1" smashed="yes">
<attribute name="NAME" x="101.9556" y="39.5986" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="96.2406" y="24.3586" size="2.0828" layer="96" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="96.2406" y="49.7586" size="2.0828" layer="96" ratio="6" rot="SR0"/>
</instance>
<instance part="IO9" gate="P" x="63.5" y="45.72" smashed="yes" rot="R180">
<attribute name="NAME" x="69.723" y="46.4058" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="64.643" y="49.022" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="BAT+" gate="P" x="83.82" y="43.18" smashed="yes">
<attribute name="NAME" x="82.677" y="45.0342" size="1.778" layer="95"/>
<attribute name="VALUE" x="82.677" y="39.878" size="1.778" layer="96"/>
</instance>
<instance part="C4" gate="G$1" x="88.9" y="58.42" smashed="yes" rot="R180">
<attribute name="NAME" x="87.884" y="57.785" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="87.884" y="62.611" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R4" gate="G$1" x="132.08" y="33.02" smashed="yes" rot="R180">
<attribute name="NAME" x="135.89" y="31.5214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="130.81" y="31.242" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R5" gate="G$1" x="132.08" y="27.94" smashed="yes" rot="R180">
<attribute name="NAME" x="135.89" y="26.4414" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="130.81" y="26.162" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C5" gate="G$1" x="129.54" y="40.64" smashed="yes" rot="R180">
<attribute name="NAME" x="133.604" y="40.005" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="136.144" y="44.831" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C6" gate="G$1" x="139.7" y="40.64" smashed="yes" rot="R180">
<attribute name="NAME" x="143.764" y="40.005" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="146.304" y="44.831" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="U3" gate="A" x="91.44" y="106.68" smashed="yes">
<attribute name="NAME" x="112.1156" y="115.7986" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="111.4806" y="113.2586" size="2.0828" layer="96" ratio="6" rot="SR0"/>
</instance>
<instance part="U4" gate="A" x="30.48" y="111.76" smashed="yes">
<attribute name="NAME" x="38.4556" y="103.0986" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="35.2806" y="98.0186" size="2.0828" layer="96" ratio="6" rot="SR0"/>
</instance>
<instance part="C7" gate="G$1" x="30.48" y="119.38" smashed="yes" rot="R270">
<attribute name="NAME" x="31.115" y="118.364" size="1.778" layer="95" rot="R270"/>
<attribute name="VALUE" x="26.289" y="118.364" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="C8" gate="G$1" x="68.58" y="104.14" smashed="yes">
<attribute name="NAME" x="69.596" y="104.775" size="1.778" layer="95"/>
<attribute name="VALUE" x="69.596" y="99.949" size="1.778" layer="96"/>
</instance>
<instance part="Q1" gate="A" x="154.94" y="88.9" smashed="yes">
<attribute name="NAME" x="162.9156" y="96.1136" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="159.7406" y="93.5736" size="2.0828" layer="96" ratio="6" rot="SR0"/>
</instance>
<instance part="R6" gate="G$1" x="154.94" y="101.6" smashed="yes" rot="R180">
<attribute name="NAME" x="153.67" y="105.1814" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="158.75" y="104.902" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R7" gate="G$1" x="154.94" y="99.06" smashed="yes" rot="R180">
<attribute name="NAME" x="153.67" y="97.5614" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="158.75" y="97.282" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R8" gate="G$1" x="63.5" y="35.56" smashed="yes" rot="R90">
<attribute name="NAME" x="67.0814" y="34.29" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="66.802" y="29.21" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R9" gate="G$1" x="60.96" y="33.02" smashed="yes" rot="R90">
<attribute name="NAME" x="59.4614" y="34.29" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="59.182" y="29.21" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R10" gate="G$1" x="83.82" y="91.44" smashed="yes" rot="R270">
<attribute name="NAME" x="80.2386" y="90.17" size="1.778" layer="95" rot="R270"/>
<attribute name="VALUE" x="80.518" y="95.25" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R11" gate="G$1" x="60.96" y="43.18" smashed="yes">
<attribute name="NAME" x="57.15" y="44.6786" size="1.778" layer="95"/>
<attribute name="VALUE" x="57.15" y="39.878" size="1.778" layer="96"/>
</instance>
<instance part="U5" gate="A" x="170.18" y="48.26" smashed="yes">
<attribute name="NAME" x="195.9356" y="57.3786" size="2.0828" layer="95" ratio="6" rot="SR0"/>
<attribute name="VALUE" x="195.3006" y="54.8386" size="2.0828" layer="96" ratio="6" rot="SR0"/>
</instance>
<instance part="C9" gate="G$1" x="248.92" y="35.56" smashed="yes" rot="R180">
<attribute name="NAME" x="247.904" y="34.925" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="247.904" y="39.751" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="C10" gate="G$1" x="256.54" y="35.56" smashed="yes" rot="R180">
<attribute name="NAME" x="255.524" y="34.925" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="255.524" y="39.751" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="3V3" gate="P" x="218.44" y="99.06" smashed="yes" rot="R180">
<attribute name="NAME" x="224.663" y="99.7458" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="219.583" y="102.362" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND" gate="P" x="218.44" y="96.52" smashed="yes" rot="R180">
<attribute name="NAME" x="224.663" y="97.2058" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="219.583" y="99.822" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="RX" gate="P" x="218.44" y="93.98" smashed="yes" rot="R180">
<attribute name="NAME" x="224.663" y="94.6658" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="219.583" y="97.282" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="TX" gate="P" x="218.44" y="91.44" smashed="yes" rot="R180">
<attribute name="NAME" x="224.663" y="92.1258" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="219.583" y="94.742" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="BAT-" gate="P" x="88.9" y="30.48" smashed="yes" rot="R90">
<attribute name="NAME" x="92.1258" y="26.797" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="92.202" y="29.337" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="L1" gate="A" x="99.06" y="55.88" smashed="yes">
<attribute name="VALUE" x="99.6188" y="59.2328" size="1.778" layer="96" ratio="10" rot="SR0"/>
<attribute name="NAME" x="98.1456" y="56.6928" size="1.778" layer="95" ratio="10" rot="SR0"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="3V3" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="2"/>
<wire x1="33.02" y1="68.58" x2="38.1" y2="68.58" width="0.1524" layer="91"/>
<wire x1="38.1" y1="68.58" x2="48.26" y2="68.58" width="0.1524" layer="91"/>
<pinref part="C2" gate="G$1" pin="1"/>
<junction x="38.1" y="68.58"/>
<pinref part="C3" gate="G$1" pin="1"/>
<pinref part="U1" gate="G$1" pin="3V3"/>
<wire x1="55.88" y1="58.42" x2="55.88" y2="68.58" width="0.1524" layer="91"/>
<wire x1="55.88" y1="68.58" x2="48.26" y2="68.58" width="0.1524" layer="91"/>
<junction x="48.26" y="68.58"/>
<wire x1="55.88" y1="68.58" x2="55.88" y2="78.74" width="0.1524" layer="91"/>
<junction x="55.88" y="68.58"/>
<label x="55.88" y="78.74" size="1.778" layer="95" rot="R90" xref="yes"/>
</segment>
<segment>
<pinref part="R2" gate="G$1" pin="1"/>
<wire x1="5.08" y1="45.72" x2="2.54" y2="45.72" width="0.1524" layer="91"/>
<label x="2.54" y="45.72" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="R3" gate="G$1" pin="2"/>
<wire x1="66.04" y1="48.26" x2="68.58" y2="48.26" width="0.1524" layer="91"/>
<label x="68.58" y="48.26" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="U2" gate="A" pin="VOUT"/>
<wire x1="139.7" y1="38.1" x2="129.54" y2="38.1" width="0.1524" layer="91"/>
<wire x1="129.54" y1="38.1" x2="124.46" y2="38.1" width="0.1524" layer="91"/>
<junction x="139.7" y="38.1"/>
<label x="147.32" y="38.1" size="1.778" layer="95" xref="yes"/>
<pinref part="C5" gate="G$1" pin="1"/>
<junction x="129.54" y="38.1"/>
<pinref part="C6" gate="G$1" pin="1"/>
<wire x1="139.7" y1="38.1" x2="147.32" y2="38.1" width="0.1524" layer="91"/>
<junction x="139.7" y="38.1"/>
<pinref part="R4" gate="G$1" pin="1"/>
<wire x1="137.16" y1="33.02" x2="139.7" y2="33.02" width="0.1524" layer="91"/>
<wire x1="139.7" y1="33.02" x2="139.7" y2="38.1" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U4" gate="A" pin="VIN"/>
<pinref part="C7" gate="G$1" pin="1"/>
<wire x1="33.02" y1="119.38" x2="33.02" y2="111.76" width="0.1524" layer="91"/>
<wire x1="33.02" y1="119.38" x2="71.12" y2="119.38" width="0.1524" layer="91"/>
<wire x1="71.12" y1="119.38" x2="71.12" y2="111.76" width="0.1524" layer="91"/>
<junction x="33.02" y="119.38"/>
<pinref part="U4" gate="A" pin="VEN"/>
<wire x1="71.12" y1="111.76" x2="68.58" y2="111.76" width="0.1524" layer="91"/>
<label x="71.12" y="111.76" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="U5" gate="A" pin="VDD"/>
<pinref part="C10" gate="G$1" pin="2"/>
<pinref part="C9" gate="G$1" pin="2"/>
<wire x1="248.92" y1="40.64" x2="256.54" y2="40.64" width="0.1524" layer="91"/>
<wire x1="228.6" y1="40.64" x2="231.14" y2="40.64" width="0.1524" layer="91"/>
<junction x="248.92" y="40.64"/>
<pinref part="U5" gate="A" pin="VDD_IO"/>
<wire x1="231.14" y1="40.64" x2="248.92" y2="40.64" width="0.1524" layer="91"/>
<wire x1="228.6" y1="43.18" x2="231.14" y2="43.18" width="0.1524" layer="91"/>
<wire x1="231.14" y1="43.18" x2="231.14" y2="40.64" width="0.1524" layer="91"/>
<junction x="231.14" y="40.64"/>
<label x="231.14" y="43.18" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="3V3" gate="P" pin="P"/>
<wire x1="215.9" y1="99.06" x2="213.36" y2="99.06" width="0.1524" layer="91"/>
<label x="213.36" y="99.06" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="R8" gate="G$1" pin="1"/>
<wire x1="63.5" y1="30.48" x2="63.5" y2="27.94" width="0.1524" layer="91"/>
<pinref part="R9" gate="G$1" pin="1"/>
<wire x1="63.5" y1="27.94" x2="60.96" y2="27.94" width="0.1524" layer="91"/>
<wire x1="63.5" y1="27.94" x2="66.04" y2="27.94" width="0.1524" layer="91"/>
<label x="66.04" y="27.94" size="1.778" layer="95" xref="yes"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="EN"/>
<wire x1="17.78" y1="55.88" x2="20.32" y2="55.88" width="0.1524" layer="91"/>
<pinref part="R1" gate="G$1" pin="1"/>
<wire x1="17.78" y1="55.88" x2="17.78" y2="68.58" width="0.1524" layer="91"/>
<wire x1="17.78" y1="68.58" x2="22.86" y2="68.58" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="1"/>
<junction x="17.78" y="68.58"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="C1" gate="G$1" pin="2"/>
<wire x1="17.78" y1="76.2" x2="17.78" y2="78.74" width="0.1524" layer="91"/>
<label x="17.78" y="78.74" size="1.778" layer="95" rot="R90" xref="yes"/>
<pinref part="C3" gate="G$1" pin="2"/>
<pinref part="C2" gate="G$1" pin="2"/>
<wire x1="48.26" y1="76.2" x2="38.1" y2="76.2" width="0.1524" layer="91"/>
<wire x1="38.1" y1="76.2" x2="17.78" y2="76.2" width="0.1524" layer="91"/>
<junction x="38.1" y="76.2"/>
<junction x="17.78" y="76.2"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="GND"/>
<wire x1="55.88" y1="25.4" x2="58.42" y2="25.4" width="0.1524" layer="91"/>
<label x="58.42" y="25.4" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="C4" gate="G$1" pin="2"/>
<wire x1="88.9" y1="63.5" x2="88.9" y2="66.04" width="0.1524" layer="91"/>
<label x="88.9" y="66.04" size="1.778" layer="95" rot="R90" xref="yes"/>
</segment>
<segment>
<pinref part="U2" gate="A" pin="GND"/>
<wire x1="88.9" y1="33.02" x2="86.36" y2="33.02" width="0.1524" layer="91"/>
<label x="86.36" y="33.02" size="1.778" layer="95" rot="R180" xref="yes"/>
<pinref part="BAT-" gate="P" pin="P"/>
<junction x="88.9" y="33.02"/>
</segment>
<segment>
<pinref part="R5" gate="G$1" pin="1"/>
<wire x1="137.16" y1="27.94" x2="139.7" y2="27.94" width="0.1524" layer="91"/>
<label x="139.7" y="27.94" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="C5" gate="G$1" pin="2"/>
<pinref part="C6" gate="G$1" pin="2"/>
<wire x1="129.54" y1="45.72" x2="139.7" y2="45.72" width="0.1524" layer="91"/>
<wire x1="139.7" y1="45.72" x2="139.7" y2="48.26" width="0.1524" layer="91"/>
<junction x="139.7" y="45.72"/>
<label x="139.7" y="48.26" size="1.778" layer="95" rot="R90" xref="yes"/>
</segment>
<segment>
<pinref part="U3" gate="A" pin="GND4"/>
<wire x1="139.7" y1="106.68" x2="142.24" y2="106.68" width="0.1524" layer="91"/>
<label x="142.24" y="106.68" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="U3" gate="A" pin="GND3"/>
<wire x1="93.98" y1="93.98" x2="88.9" y2="93.98" width="0.1524" layer="91"/>
<wire x1="88.9" y1="93.98" x2="88.9" y2="99.06" width="0.1524" layer="91"/>
<pinref part="U3" gate="A" pin="GND2"/>
<wire x1="93.98" y1="99.06" x2="88.9" y2="99.06" width="0.1524" layer="91"/>
<pinref part="U3" gate="A" pin="GND"/>
<wire x1="93.98" y1="101.6" x2="88.9" y2="101.6" width="0.1524" layer="91"/>
<wire x1="88.9" y1="101.6" x2="88.9" y2="99.06" width="0.1524" layer="91"/>
<junction x="88.9" y="99.06"/>
<label x="88.9" y="93.98" size="1.778" layer="95" rot="R270" xref="yes"/>
<pinref part="U3" gate="A" pin="AVSSVCSEL"/>
<wire x1="93.98" y1="104.14" x2="88.9" y2="104.14" width="0.1524" layer="91"/>
<wire x1="88.9" y1="104.14" x2="88.9" y2="101.6" width="0.1524" layer="91"/>
<junction x="88.9" y="101.6"/>
</segment>
<segment>
<pinref part="U4" gate="A" pin="GND"/>
<wire x1="33.02" y1="109.22" x2="25.4" y2="109.22" width="0.1524" layer="91"/>
<label x="22.86" y="109.22" size="1.778" layer="95" rot="R180" xref="yes"/>
<pinref part="C7" gate="G$1" pin="2"/>
<wire x1="25.4" y1="109.22" x2="22.86" y2="109.22" width="0.1524" layer="91"/>
<wire x1="25.4" y1="119.38" x2="25.4" y2="109.22" width="0.1524" layer="91"/>
<junction x="25.4" y="109.22"/>
</segment>
<segment>
<pinref part="C8" gate="G$1" pin="2"/>
<wire x1="68.58" y1="99.06" x2="68.58" y2="96.52" width="0.1524" layer="91"/>
<label x="68.58" y="96.52" size="1.778" layer="95" rot="R270" xref="yes"/>
</segment>
<segment>
<pinref part="U5" gate="A" pin="GND_3"/>
<wire x1="228.6" y1="35.56" x2="231.14" y2="35.56" width="0.1524" layer="91"/>
<wire x1="231.14" y1="35.56" x2="231.14" y2="38.1" width="0.1524" layer="91"/>
<pinref part="U5" gate="A" pin="GND"/>
<wire x1="231.14" y1="38.1" x2="228.6" y2="38.1" width="0.1524" layer="91"/>
<label x="231.14" y="38.1" size="1.778" layer="95" xref="yes"/>
<wire x1="170.18" y1="27.94" x2="231.14" y2="27.94" width="0.1524" layer="91"/>
<wire x1="231.14" y1="27.94" x2="231.14" y2="35.56" width="0.1524" layer="91"/>
<junction x="231.14" y="35.56"/>
<pinref part="U5" gate="A" pin="GND_2"/>
<pinref part="U5" gate="A" pin="RES"/>
<wire x1="172.72" y1="38.1" x2="170.18" y2="38.1" width="0.1524" layer="91"/>
<wire x1="170.18" y1="38.1" x2="170.18" y2="35.56" width="0.1524" layer="91"/>
<wire x1="170.18" y1="35.56" x2="172.72" y2="35.56" width="0.1524" layer="91"/>
<wire x1="170.18" y1="27.94" x2="170.18" y2="35.56" width="0.1524" layer="91"/>
<junction x="170.18" y="35.56"/>
<pinref part="C9" gate="G$1" pin="1"/>
<wire x1="248.92" y1="33.02" x2="248.92" y2="27.94" width="0.1524" layer="91"/>
<pinref part="C10" gate="G$1" pin="1"/>
<wire x1="248.92" y1="27.94" x2="256.54" y2="27.94" width="0.1524" layer="91"/>
<wire x1="256.54" y1="27.94" x2="256.54" y2="33.02" width="0.1524" layer="91"/>
<wire x1="248.92" y1="27.94" x2="231.14" y2="27.94" width="0.1524" layer="91"/>
<junction x="248.92" y="27.94"/>
<junction x="231.14" y="27.94"/>
</segment>
<segment>
<pinref part="GND" gate="P" pin="P"/>
<wire x1="215.9" y1="96.52" x2="213.36" y2="96.52" width="0.1524" layer="91"/>
<label x="213.36" y="96.52" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
</net>
<net name="TXD" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="TXD"/>
<wire x1="20.32" y1="30.48" x2="17.78" y2="30.48" width="0.1524" layer="91"/>
<label x="17.78" y="30.48" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="TX" gate="P" pin="P"/>
<wire x1="215.9" y1="91.44" x2="213.36" y2="91.44" width="0.1524" layer="91"/>
<label x="213.36" y="91.44" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
</net>
<net name="RXD" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="RXD"/>
<wire x1="20.32" y1="27.94" x2="17.78" y2="27.94" width="0.1524" layer="91"/>
<label x="17.78" y="27.94" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="RX" gate="P" pin="P"/>
<wire x1="215.9" y1="93.98" x2="213.36" y2="93.98" width="0.1524" layer="91"/>
<label x="213.36" y="93.98" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="IO2"/>
<pinref part="R2" gate="G$1" pin="2"/>
<wire x1="15.24" y1="45.72" x2="20.32" y2="45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="IO8"/>
<pinref part="R3" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="IO9"/>
<wire x1="60.96" y1="45.72" x2="55.88" y2="45.72" width="0.1524" layer="91"/>
<pinref part="IO9" gate="P" pin="P"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="BAT+" gate="P" pin="P"/>
<pinref part="U2" gate="A" pin="VIN"/>
<wire x1="86.36" y1="43.18" x2="88.9" y2="43.18" width="0.1524" layer="91"/>
<wire x1="99.06" y1="55.88" x2="88.9" y2="55.88" width="0.1524" layer="91"/>
<wire x1="88.9" y1="55.88" x2="88.9" y2="43.18" width="0.1524" layer="91"/>
<junction x="88.9" y="43.18"/>
<pinref part="C4" gate="G$1" pin="1"/>
<junction x="88.9" y="55.88"/>
<pinref part="U2" gate="A" pin="EN"/>
<wire x1="88.9" y1="38.1" x2="88.9" y2="43.18" width="0.1524" layer="91"/>
<pinref part="L1" gate="A" pin="2"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<wire x1="114.3" y1="55.88" x2="124.46" y2="55.88" width="0.1524" layer="91"/>
<pinref part="U2" gate="A" pin="SW"/>
<wire x1="124.46" y1="55.88" x2="124.46" y2="43.18" width="0.1524" layer="91"/>
<pinref part="L1" gate="A" pin="1"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="R4" gate="G$1" pin="2"/>
<pinref part="U2" gate="A" pin="FB"/>
<wire x1="127" y1="33.02" x2="124.46" y2="33.02" width="0.1524" layer="91"/>
<pinref part="R5" gate="G$1" pin="2"/>
<wire x1="127" y1="27.94" x2="127" y2="33.02" width="0.1524" layer="91"/>
<junction x="127" y="33.02"/>
</segment>
</net>
<net name="2V8" class="0">
<segment>
<pinref part="U4" gate="A" pin="VOUT"/>
<pinref part="C8" gate="G$1" pin="1"/>
<wire x1="68.58" y1="106.68" x2="73.66" y2="106.68" width="0.1524" layer="91"/>
<junction x="68.58" y="106.68"/>
<label x="73.66" y="106.68" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="Q1" gate="A" pin="2"/>
<wire x1="157.48" y1="83.82" x2="154.94" y2="83.82" width="0.1524" layer="91"/>
<label x="154.94" y="83.82" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="Q1" gate="A" pin="5"/>
<wire x1="180.34" y1="83.82" x2="182.88" y2="83.82" width="0.1524" layer="91"/>
<label x="182.88" y="83.82" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="R7" gate="G$1" pin="1"/>
<pinref part="R6" gate="G$1" pin="1"/>
<wire x1="160.02" y1="99.06" x2="160.02" y2="101.6" width="0.1524" layer="91"/>
<label x="160.02" y="101.6" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="R10" gate="G$1" pin="2"/>
<wire x1="83.82" y1="86.36" x2="83.82" y2="83.82" width="0.1524" layer="91"/>
<label x="83.82" y="83.82" size="1.778" layer="95" rot="R270" xref="yes"/>
</segment>
<segment>
<pinref part="U3" gate="A" pin="AVDDVCSEL"/>
<pinref part="U3" gate="A" pin="AVDD"/>
<wire x1="88.9" y1="106.68" x2="93.98" y2="106.68" width="0.1524" layer="91"/>
<wire x1="139.7" y1="104.14" x2="152.4" y2="104.14" width="0.1524" layer="91"/>
<wire x1="152.4" y1="104.14" x2="152.4" y2="119.38" width="0.1524" layer="91"/>
<wire x1="152.4" y1="119.38" x2="88.9" y2="119.38" width="0.1524" layer="91"/>
<wire x1="88.9" y1="119.38" x2="88.9" y2="106.68" width="0.1524" layer="91"/>
<label x="88.9" y="119.38" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<wire x1="144.78" y1="73.66" x2="180.34" y2="73.66" width="0.1524" layer="91"/>
<pinref part="Q1" gate="A" pin="4"/>
<wire x1="180.34" y1="73.66" x2="180.34" y2="78.74" width="0.1524" layer="91"/>
<pinref part="R7" gate="G$1" pin="2"/>
<pinref part="U3" gate="A" pin="SDA"/>
<wire x1="139.7" y1="99.06" x2="144.78" y2="99.06" width="0.1524" layer="91"/>
<wire x1="144.78" y1="99.06" x2="149.86" y2="99.06" width="0.1524" layer="91"/>
<wire x1="144.78" y1="73.66" x2="144.78" y2="99.06" width="0.1524" layer="91"/>
<junction x="144.78" y="99.06"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="Q1" gate="A" pin="1"/>
<wire x1="157.48" y1="88.9" x2="147.32" y2="88.9" width="0.1524" layer="91"/>
<pinref part="U3" gate="A" pin="SCL"/>
<wire x1="149.86" y1="101.6" x2="147.32" y2="101.6" width="0.1524" layer="91"/>
<pinref part="R6" gate="G$1" pin="2"/>
<wire x1="147.32" y1="101.6" x2="139.7" y2="101.6" width="0.1524" layer="91"/>
<wire x1="147.32" y1="88.9" x2="147.32" y2="101.6" width="0.1524" layer="91"/>
<junction x="147.32" y="101.6"/>
</segment>
</net>
<net name="SDA" class="0">
<segment>
<pinref part="Q1" gate="A" pin="3"/>
<wire x1="157.48" y1="78.74" x2="154.94" y2="78.74" width="0.1524" layer="91"/>
<label x="154.94" y="78.74" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="IO18"/>
<wire x1="55.88" y1="40.64" x2="63.5" y2="40.64" width="0.1524" layer="91"/>
<label x="66.04" y="40.64" size="1.778" layer="95" xref="yes"/>
<pinref part="R8" gate="G$1" pin="2"/>
<wire x1="63.5" y1="40.64" x2="66.04" y2="40.64" width="0.1524" layer="91"/>
<junction x="63.5" y="40.64"/>
</segment>
</net>
<net name="SCL" class="0">
<segment>
<pinref part="Q1" gate="A" pin="6"/>
<wire x1="180.34" y1="88.9" x2="182.88" y2="88.9" width="0.1524" layer="91"/>
<label x="182.88" y="88.9" size="1.778" layer="95" xref="yes"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="IO19"/>
<wire x1="55.88" y1="38.1" x2="60.96" y2="38.1" width="0.1524" layer="91"/>
<label x="66.04" y="38.1" size="1.778" layer="95" xref="yes"/>
<pinref part="R9" gate="G$1" pin="2"/>
<wire x1="60.96" y1="38.1" x2="66.04" y2="38.1" width="0.1524" layer="91"/>
<junction x="60.96" y="38.1"/>
</segment>
</net>
<net name="XSHUT" class="0">
<segment>
<pinref part="U3" gate="A" pin="XSHUT"/>
<wire x1="93.98" y1="96.52" x2="83.82" y2="96.52" width="0.1524" layer="91"/>
<label x="83.82" y="96.52" size="1.778" layer="95" rot="R90" xref="yes"/>
<pinref part="R10" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="R11" gate="G$1" pin="2"/>
<wire x1="66.04" y1="43.18" x2="68.58" y2="43.18" width="0.1524" layer="91"/>
<label x="68.58" y="43.18" size="1.778" layer="95" xref="yes"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="IO10"/>
<pinref part="R11" gate="G$1" pin="1"/>
</segment>
</net>
<net name="INT" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="IO0"/>
<wire x1="20.32" y1="50.8" x2="17.78" y2="50.8" width="0.1524" layer="91"/>
<label x="17.78" y="50.8" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="U5" gate="A" pin="INT1"/>
<wire x1="228.6" y1="48.26" x2="231.14" y2="48.26" width="0.1524" layer="91"/>
<label x="231.14" y="48.26" size="1.778" layer="95" xref="yes"/>
</segment>
</net>
<net name="SPC" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="IO1"/>
<wire x1="20.32" y1="48.26" x2="17.78" y2="48.26" width="0.1524" layer="91"/>
<label x="17.78" y="48.26" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="U5" gate="A" pin="SCL/SPC"/>
<wire x1="172.72" y1="48.26" x2="170.18" y2="48.26" width="0.1524" layer="91"/>
<label x="170.18" y="48.26" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
</net>
<net name="MISO" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="IO4"/>
<wire x1="20.32" y1="40.64" x2="17.78" y2="40.64" width="0.1524" layer="91"/>
<label x="17.78" y="40.64" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="U5" gate="A" pin="SDO/SA0"/>
<wire x1="172.72" y1="43.18" x2="170.18" y2="43.18" width="0.1524" layer="91"/>
<label x="170.18" y="43.18" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
</net>
<net name="MOSI" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="IO5"/>
<wire x1="20.32" y1="38.1" x2="17.78" y2="38.1" width="0.1524" layer="91"/>
<label x="17.78" y="38.1" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="U5" gate="A" pin="SDA/SDI/SDO"/>
<wire x1="172.72" y1="40.64" x2="170.18" y2="40.64" width="0.1524" layer="91"/>
<label x="170.18" y="40.64" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
</net>
<net name="CS" class="0">
<segment>
<pinref part="U5" gate="A" pin="CS"/>
<wire x1="172.72" y1="45.72" x2="170.18" y2="45.72" width="0.1524" layer="91"/>
<label x="170.18" y="45.72" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
<segment>
<pinref part="U1" gate="G$1" pin="IO3"/>
<wire x1="20.32" y1="43.18" x2="17.78" y2="43.18" width="0.1524" layer="91"/>
<label x="17.78" y="43.18" size="1.778" layer="95" rot="R180" xref="yes"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
<note version="8.4" severity="warning">
Since Version 8.4, EAGLE supports properties for SPICE simulation. 
Probes in schematics and SPICE mapping objects found in parts and library devices
will not be understood with this version. Update EAGLE to the latest version
for full support of SPICE simulation. 
</note>
</compatibility>
</eagle>
