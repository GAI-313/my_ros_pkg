# my_robot ~自作パッケージでロボットを作る~

　自分でロボットのパッケージを作って遊んでみる。ws ディレクトリとその中に src ディレクトリを作成し、
catkin_create_pkg コマンドで自分のパッケージを作成する。
```bash
catkin_create_pkg <pkg_name> <依存関係pkg> <依存関係pkg> ...
```
- <pkg_name> には自分の好きなパッケージの名前にする。ここでは my_robot という名前にした。
- <依存関係pkg> には、これから作成するパッケージに関わる他のパッケージまたはライブラリたちを指定する。
基本的に ROS 環境内で Python と c++ を使うには、rospy と roscpp というライブラリが必要となるので、必ずこれを記述しよう。
また、ROS のトピック通信のメッセージ種類として最も基本な std_msgs を今回は指定する。ほかにもメッセージの種類はたくさんある。

　パッケージ名を my_robot。初期に導入する依存関係たちは rospy、roscpp、std_msgs としてパッケージを作るとするならば、以下のコマンドでできる。
```bash
catkin_create_pkg my_robot rospy roscpp std_msgs
```
　すると、以下のようなログが表示される。以下のログの最後の行のような Successfully と書かれていれば自作パッケージの作成は完了である。
```bash
$ catkin_create_pkg my_robot rospy roscpp std_msgs
Created file my_robot/package.xml
Created file my_robot/CMakeLists.txt
Created folder my_robot/include/my_robot
Created folder my_robot/src
Successfully created files in /home/gai/catkin_ws/src/my_robot. Please adjust the values in package.xml.
```
　そしたら ws ディレクトリに移動して catkin build を実行する。
昔は catkin_make コマンドが主流だったが、これより改善された catkin build がおすすめである。
```bash
catkin build
```
　ビルドが成功したら、以下のコマンドでセットアップを通しておこう。
```bash
source devel/setup.bash
```
　そしたら、ROS のコマンドの一部である。 roscd コマンドを使って、先ほど作成した自作パッケージのディレクトリまで移送してみよう。
```bash
roscd my_robot
```
　作成した自作パッケージのディレクトリまで移動できたら ROS にも自作パッケージがちゃんと認識されていることとなる。
