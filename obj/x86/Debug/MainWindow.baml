   M S B A M L   `   `   `  ����  	KinectOSC    KinectOSC.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  
MainWindow��5   	   $	��755��6   $	��755��6)   $	��230��69   $	��680��6I   $	��680��6U   $	��680��6d    � 
ResizeMode$ CanResizeWithGrip=�5   	    � 	Resources 5         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType  SquareCheckBox �� Property �� Value �� 
TargetType �� Orientation	 �Name
 ��Name ��Name �� TextAlignment �� Triggers P� Property P� Value �� 
TargetName��%�  &	        &	 #     &	 F     &	 i     7  7  �� 5   
   $ 	#ff6e6e6e��62   �� 5   
   $ 	#ff52318f��64   �� 5	   
   $ 	#ff00BCF2��62   �� 5
   
   " ��6   .�5      ��  � �� 5      �� 5      " ��6&   .R�5      � .+�5      �� $ 
Horizontal=�6)   
������ .+�5   "   � $	 SquareCheckBoxChecked��6'   .+�5   &   �� $
 CheckedNormal��6,   $��Images\CheckedNormal.png��6C   $
��None=�6e   $��Center=�6t   �� 5   &   $
 CheckedHover��6,   $��Images\CheckedHover.png��6B   $
��None=�6c   $��Center=�6r   $y�	Collapsed=�6�   5   #   � 5   "   $	 SquareCheckBoxUnchecked��6'   $y�	Collapsed=�6H   .+�5   &   �� $
 UncheckedNormal��6,   $ ��Images\UncheckedNormal.png��6E   $
��None=�6i   $��Center=�6x   �� 5   &   $
 UncheckedHover��6,   $��Images\UncheckedHover.png��6D   $
��None=�6g   $��Center=�6v   $y�	Collapsed=�6�   5   #   �� 5   "   $ SquareCheckBoxText��6,   #��z��6H   $
 Left=�6i   $��Center=�6   8��  $��15(�6�   $��9,0,0,0q�6�   5      5       5      P� 5       �@d�	IsChecked$ falseY�6;   . �5   "   �� $ SquareCheckBoxChecked��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5   "   $ SquareCheckBoxUnchecked��6O    � y�$ Visible=�6?   6O   5      P� 5        � {� .  . �5!   "   �� $ SquareCheckBoxChecked��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5"   "   $ SquareCheckBoxUnchecked��6N    � y�$ Hidden=�6?   6N   �� 5#   "   $ SquareCheckBoxText��6P    � ��
 �����5$      P� 5%       �@J�IsMouseOver . . �5&   "   �� $ CheckedNormal��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5'   "   $ UncheckedNormal��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5(   "   $ CheckedHover��6O    � y�$ Visible=�6?   6O   �� 5)   "   $ UncheckedHover��6O    � y�$ Visible=�6?   6O   �� 5*   "   8  $ SquareCheckBoxText��6h    � ��5+      5-      5/      50      51      .��53      � -   5        $��
layoutGrid��53      $��	10 0 10 0q�6    � RowDefinitions 54   
   �� 55      $
��Auto �6   �� 56      $��* �6   �� 57      $
��Auto �6   �� 58      $
��Auto �6    � ColumnDefinitions 5:   
   �� 5;      $��* �6   �� 5<      $��* �6   �� 5=      $��* �6   .+�5?   
   ]� $��0��6   $��3��6!   $��0 0 0 20q�65   .+�5@      �� $
��Left=�6   $��Images\Logo.png��6*   $
��None=�6C   $��0 10 0 5q�6R   �� 5A      $��Right=�6   $��Bottom=�6/   #��[  6J   $��Segoe UI)�6x   $��18(�6�   Skeleton OSC6�   �� 5B      $��1��6   $��Images\Status.png��6$   $
��None=�6?   $��Center=�6N   $��0 0 0 5q�6k   5C      �� 5D   
   $��1��6   $��3��6"   $	��640��66   $	��480��6B   $
��Left=�6O   
�������.+�5E      �� -   5        $��
ColorImage��5E      $	��640��6&   $	��480��62   $��0,0,0,0q�6?   �� 5F      -   5        $��Image��5F      $	��640��6!   $	��480��6-   $��
0,-480,0,0q�6:    K� ZIndex$	 100��6N   �� 5G      -   5        $��	oscViewer��5I      $
��Auto=�5G      $y�	Collapsed=�6H   $
��Auto=�6_   ���	#BBFFFFFF$��16(�5H   -   ��. $	��480��6a   $��Right=�6n   $��
0,-480,0,0q�6�   $	��Top=�5I   '   $	��355��6?   $	 200��6K   �� 5J      -   5        $��CloseOscViewer��5J   ,   $y�	Collapsed=�6   $��Close��6B   $��Right=�6p   $	��Top=�6�   $��0 -480 20 0q�6�   $	��100��6�   $��30��6�   $��16(�5K      $	 300��6#   68   5L      �� 5M   
   $��2��6   .+�5N      �� -   5        $��checkBoxSeatedMode��5O   &   #��[ 5N      $��Seated Mode��6?    d� 	IsChecked$ FalseY�6U   $
��Auto��6g   $
��Left=�6u   $��Center=�6�   $��	0 0 10 10q�5O       ^� ShowDuration$ 12000��6�    � ToolTip 5P      _� 5Q      .��5R      �� .+�5S      �� $
��Bold"�6(   Toggle Seated Mode6N   �� 5T      USIn seated mode you only get joints for the arms, hands, neck, head, spine (no legs)6}   5U      5V      5X      �� 5Y      -   5        $��checkBoxTrackNearestOnly��5Z   (   #��[ 5Y      $��Track Nearest Only��6?   $ FalseY�6\   $
��Auto��6n   $
��Left=�6|   $��Center=�6�   $��0 -10 10 10q�5Z      $ 12000��6�    5[      _� 5\      .��5]      �� .+�5^      �� $
��Bold"�6(   Toggle Track Nearest Only6U   �� 5_      =;If checked: the Kinect will only track the nearest skeleton6e   �� 5`      LJIf unchecked: the Kinect will track two skeletons as soon as they are seen6t   5a      5b      5d      �� 5e      -	   5        $��checkBoxDrawColor��5f      #��[ 5e      $
 TrueY�6?   $��
Show Video��6P   $
��Auto��6e   $
��Left=�6s   $��Center=�6�   $��0 -10 10 10q�6�    5g      _� 5h      .��5i      �� .+�5j      �� $
��Bold"�6(   Toggle Video Streaming6R   �� 5k      +)If checked: Video will be shown in window6S   �� 5l      <:If unchecked: Video window will show the last frame frozen6d   5m      5n      5p      �� 5q      -
   5        $��checkBoxShowSkeleton��5r      #��[ 5q      $
 TrueY�6?   $��Show Skeleton��6P   $
��Auto��6h   $
��Left=�6v   $��Center=�6�   $��0 -10 10 10q�6�    5s      _� 5t      .��5u      �� .+�5v      �� $
��Bold"�6(   Toggle Skeleton Drawing6S   �� 5w      %#If checked: Skeletons will be drawn6M   �� 5x      +)If unchecked: Skeletons will NOT be drawn6S   5y      5z      5|      5}      �� 5~   
   $��2��6   $��2��6"   $��1��66   .+�5      �� -   5        $��checkBoxOscOn��5�      #��[ 5      $
 TrueY�6?   $��OSC On��6P   $
��Auto��6a   $
��Left=�6o   $	��Top=�6�   $��	0 0 10 10q�6�    5�      _� 5�      .��5�      �� .+�5�      �� $
��Bold"�6(   Toggle OSC streaming6P   �� 5�      If checked: OSC is streaming6F   �� 5�      $"If unchecked: OSC is NOT streaming6L   5�      5�      5�      �� 5�      -   5        $��OscLabel��5�      $��OSC Address��6(   $��0,0,0,0q�6;   $��14(�6L   $��85��6Z   $
��Left=�6e   $ 24000��6�    5�      _� 5�      .��5�      �� .+�5�      �� $
��Bold"�6(   Set Osc Address Prefix6R   �� 5�      %#Osc Address should start with a "/"6M   �� 5�      xvThe OSC addresses will be whatever you put here + a counter for each data point in the order of the joints and x, y, z6�   �� 5�      VTThis is useful for programs that have hard-coded address requirements (like Isadora)6~   �� 5�      FDIf you want to let the program define the addresses leave this blank6n   �� 5�      42This only applies if "Send Separately" is checked.6\   �� 5�      DBHitting the change button or the enter key will trigger the change6l   5�      5�      5�      �� 5�      -   5        $��
OscAddress��5�      $
��Left=�6(   $	��Top=�6C   $��
85,-20,0,0q�6[   $	��110��6o   $��25��6{   $��16(�6�   $�� ��6�   $��Center=�5�      {�.  6`   �� 5�      -   5        $��ChangeAddress��5�      $��Change��6*   $��50��6;   $��25��6F   $
��Left=�6R   $��200 -25 0 0q�6�   6�   �� 5�      -   5        $��checkBoxShowOscData��5�   )   #��[ 5�      $ FalseY�6?   $��Show OSC Data��6Q   $
��Auto��6i   $
��Left=�6w   $	��Top=�6�   $��265 -55 0 10q�5�      $ 12000��6�    5�      _� 5�      .��5�      �� .+�5�      �� $
��Bold"�6(   Show OSC data window6P   �� 5�      ][You can see one frame of OSC data to verify that data is working and to see the joint names6�   5�      5�      5�      �� 5�      -   5        $&�� checkBoxSendPositionAsPercentage��5�   )   #��[ 5�      $
 TrueY�6?   $��Send Pos as %��6P   $
��Auto��6h   $
��Left=�6v   $	��Top=�6�   $��265 -31 0 10q�5�      $ 12000��6�    5�      _� 5�      .��5�      �� .+�5�      �� $
��Bold"�6(   %#Send Position Values as Percentages6_   �� 5�      nlIf checked: The values of x and y are sent as percentage of width and height, z is in meters from the camera6�   �� 5�      '%If unchecked: The values are sent raw6O   �� 5�      %#Generally this is best left checked6M   5�      5�      5�      �� 5�      -   5        $��checkBoxSendAsString��5�   )   #��[ 5�      $ FalseY�6?   $��Send As String��6Q   $
��Auto��6j   $
��Left=�6x   $	��Top=�6�   $��265 -10 0 10q�5�      $ 12000��6�    5�      _� 5�      .��5�      �� .+�5�      �� $
��Bold"�6(   OSC data format6K   �� 5�      _]If checked: OSC can be sent as one single string "Joint Name|Joint Number:x|y|z, more joints"6�   �� 5�      vtIf unchecked: Each joint can be sent separately as 3 floats (x, y, z) or totally separately using the checkbox below6�   5�      5�      5�      �� 5�      -   5        $��checkBoxSendSeparately��5�   )   #��[ 5�      $ FalseY�6?   $��Send Separately��6Q   $
��Auto��6k   $
��Left=�6y   $	��Top=�6�   $��265 -10 0 10q�5�      $ 15000��6�   {�.  5�      _� 5�      .��5�      �� .+�5�      �� $
��Bold"�6(    Toggle Sending data separately6Z   �� 5�      caIf checked: OSC will be sent as individual osc data elements "/skeleton#/JointName/x/#.##" etc...6�   �� 5�      GEThis is useful for sending to software that can't handle osc bundles.6o   �� 5�      HFIf unchecked: Each joint will be sent separately as 3 floats (x, y, z)6p   5�      5�      5�      �� 5�      -   5        $��	PortLabel��5�      $��Port Number��6)   $��	0,-40,0,0q�6<   $��14(�6O   $��25��6]   $��85��6i   $
��Left=�6t   $ 15000��6�    5�      _� 5�      .��5�      �� .+�5�      �� $
��Bold"�6(   Set Port Number6K   �� 5�      ,*Port number can be anything from 0 - 655356T   �� 5�      <:But it is recommended to use a port between 1025 and 491516d   �� 5�      DBHitting the change button or the enter key will trigger the change6l   5�      5�      5�      �� 5�      -   5        $��OscPort��5�      $
��Left=�6%   $	��Top=�6@   $��135,-35,0,0q�6X   $��60��6m   $��25��6x   $��16(�6�   $
��3001��6�   $��Center=�5�      6K   �� 5�      -   5        $��
ChangePort��5�      $��Change��6'   $��50��68   $��25��6C   $
��Left=�6O   $��200 -45 0 0q�6�   6�   5�      �� 5�   
   -   5        $��	statusBar��5�   S   $��3��6   $��2��6!   $��Stretch=�65   $��Bottom=�6d   
�������#��[  6�   .@�5�      �� $��0 0 0 10q�6   .��5�      �� -   5        $��statusBarText��5�      $��0q�61   ><Hover over the checkboxes and labels to see additional info.6z   5�      5�      5�      5�      