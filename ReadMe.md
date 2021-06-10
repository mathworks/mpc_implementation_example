# モデル予測制御　設計実装ワークフロー紹介
# 目的


当サンプルモデルは、モデル予測制御（MPC）の設計と実装のワークフローを分かりやすく紹介するための資料である。




特にマイクロコントローラに実装するときの検討事項について詳しくまとめている。


# 必要なツールボックス


本モデルでは、以下のツールボックスを利用する。ただし、インストールしていなくても、モデルを実行せず閲覧するだけであれば可能である。また、例題によっては使わないツールボックスもある。



   -  MATLAB® 
   -  Simulink® 
   -  Control System Toolbox™ 
   -  Model Predictive Control Toolbox™ 
   -  Simulink Control Design™ 
   -  Simscape™, Simscape™ Multibody™ 
   -  Symbolic Math Toolbox™ 
   -  Navigation Toolbox™ 
   -  Image Processing Toolbox™, Computer Vision Toolbox™ 
   -  Automated Driving Toolbox™ 
   -  MATLAB Coder™, Simulink Coder™, Embedded Coder® 

# 目次
## 1.一般的な紹介資料


MPCについて初学者であり、以下の資料を見ていない場合は、本サンプルモデルより先に以下の資料に目を通しておくこと。


### ビデオ 

   -  [モデル予測制御（MPC）とは？ Part1 ～ 基本的な考え方](https://www.youtube.com/watch?v=i68MkFz9L38) 
   -  [モデル予測制御（MPC）とは？ Part2 ～ Model Predictive Control Toolbox例題](https://www.youtube.com/watch?v=47LzXHOXwtU) 
   -  [応用 ～ アダプティブ・クルーズ・コントロールとセンサーフュージョン](https://www.youtube.com/watch?v=Nb3aTJ8Wgk8) 
   -  [モデル予測制御（MPC）入門](https://jp.mathworks.com/videos/cda-model-predictive-control-90293.html) 
   -  [Understanding Model Predictive Control](https://jp.mathworks.com/videos/series/understanding-model-predictive-control.html) 

### 技術紹介記事

   -  [モデル予測制御](https://jp.mathworks.com/discovery/model-predictive-control.html) 

### ホワイトペーパー


以下の資料はMPCの技術的な資料である。ある程度ツールの使い方を理解した後に読むこと。



   -  [MPC高速化のための3つの方法](https://www.mathworks.com/content/dam/mathworks/white-paper/3-ways-to-speed-up-model-predictive-controllers.pdf) 

## 2.線形MPC


最初に線形MPCを使った例を紹介する。プラントモデルとして、単純なSISOの不安定システムを用いる。以下のライブエディターを開いて作業を開始すること。




[線形MPCコントローラの設計と実装](/MPC_imple_PJ/Linear/Linear_MPC_Design_md.md)


  
## 3.陽的MPC


線形近似されたDCモーターモデルに対して陽的MPCを設計し、実装する。線形MPCと陽的MPCの比較を行う。以下のライブエディターを開いて作業を開始すること。




[陽的MPCコントローラの設計と実装](/MPC_imple_PJ/Explicit/Explicit_MPC_Design_md.md)


  
## 4.適応MPC


適応MPCの設計と実装について、4輪走行車両の制御を例に紹介する。プラントモデルとしては、等価二輪モデルを用いる。以下のライブエディターを開いて作業を開始すること。




[適応MPCコントローラの設計と実装](/MPC_imple_PJ/Adaptive/Adaptive_MPC_Design_md.md)


  
## 5.ゲインスケジュールMPC


適応MPCで用いたプラントモデルと設計手法を転用し、ゲインスケジュールMPCを設計、実装する。以下のライブエディターを開いて作業を開始すること。




[ゲインスケジュールMPCコントローラの設計と実装](/MPC_imple_PJ/Multiple/Multiple_MPC_Design_md.md)


  
## 6.非線形MPC


非線形MPCの設計と実装について、シンプルな車両モデルの運転制御を例に紹介する。以下のライブエディターを開いて作業を開始すること。




[非線形MPCコントローラの設計と実装](/MPC_imple_PJ/Nonlinear/Nonlinear_MPC_design_md.md)


  
## 7.適応MPCに内点法ソルバー適用


適応MPCのソルバーとして、内点法を用いた場合の設計と実装について、4輪走行車両の制御を例に紹介する。プラントモデルとしては、等価二輪モデルを用いる。以下のライブエディターを開いて作業を開始すること。




[適応MPCコントローラのQPソルバーの設定について検討](/MPC_imple_PJ/Adaptive/Adaptive_MPC_QP_investigation_md.md)


  
## 8.マルチステージの非線形MPC


マルチステージの非線形MPCは、コスト関数を予測ホライズンの各ステップごとに設定できる。それにより、通常の非線形MPCよりも高速に計算できる。以下のライブエディターを開いて作業を開始すること。




[マルチステージ非線形MPCの設計と実装](/MPC_imple_PJ/Nonlinear/Nonlinear_MultiStage_MPC_design_md.md)


  
## Ex.1.適応モデル予測制御による倒立制御走行ロボットの倒立制御


適応MPCを用いて、倒立振子型の自律走行ロボットの制御を構築する。物理モデルをSimscape Multibodyで構築し、それを制御する。以下のライブエディターを開いて作業を開始すること。




[適応モデル予測制御による倒立制御走行ロボットの倒立制御](/MPC_imple_PJ/InvertedPendulumRobot/InvertedPendulumRobot_design_md.md)


  
## Ex.2.モデル予測制御ならびにカメラ画像によるターゲット認識を利用した衝突回避


適応MPCを用いて、自律走行車両の制御を構築する。Automated Driving Toolboxを用いた走行環境から障害物を認識し、回避しながら走行制御を行う。以下のライブエディターを開いて作業を開始すること。




[モデル予測制御ならびにカメラ画像によるターゲット認識を利用した衝突回避](/MPC_imple_PJ/obstacleAvoid/liveScriptForObstacleAvoid_md.md)


  
# 過去バージョン


過去のバージョンのファイル一式は、以下から得ることができる。ただし、過去のモデルには、古い時期に作成したサンプルしか含まれていないことに注意すること。




GitHubからクローンしている場合には、以下の該当バージョンに戻すことで、過去バージョンファイルを得ることができる。


  


R2020b: [v2.2](https://github.com/mathworks/mpc_implementation_example/archive/refs/tags/v2.2.zip)




R2020a: [v1.1.1](https://github.com/mathworks/mpc_implementation_example/archive/refs/tags/v1.1.1.zip)



