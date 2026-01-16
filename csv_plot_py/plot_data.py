import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from matplotlib.gridspec import GridSpec

def plot_motor_data_comparison(rx_csv_file, strategy_csv_file):
    """
    同时读取RX和Strategy两个CSV文件并对比绘制数据
    生成三张图：IMU四元数、电机位置、电机速度
    只显示最大值和最小值
    每张图都是多行一列布局
    """
    try:
        # 检查文件是否存在
        if not os.path.exists(rx_csv_file):
            print(f"错误：RX数据文件 {rx_csv_file} 不存在")
            return
        if not os.path.exists(strategy_csv_file):
            print(f"错误：Strategy数据文件 {strategy_csv_file} 不存在")
            return
        
        # 读取两个CSV文件
        print(f"正在读取RX数据文件: {rx_csv_file}")
        df_rx = pd.read_csv(rx_csv_file)
        print(f"正在读取Strategy数据文件: {strategy_csv_file}")
        df_strategy = pd.read_csv(strategy_csv_file)
        
        print(f"RX数据: {len(df_rx)} 行")
        print(f"Strategy数据: {len(df_strategy)} 行")
        
        # 显示数据基本信息
        print("\nRX数据列信息:")
        print(df_rx.columns.tolist())
        print("\nStrategy数据列信息:")
        print(df_strategy.columns.tolist())
        
        # 计算相对时间（以秒为单位）
        time_rx = df_rx['timestamp'] - df_rx['timestamp'].iloc[0]
        time_strategy = df_strategy['timestamp'] - df_strategy['timestamp'].iloc[0]
        
        # 创建三张大图：IMU四元数、电机位置、电机速度
        fig_imu = plt.figure(figsize=(50, 16))
        fig_pos = plt.figure(figsize=(50, 20))
        fig_vel = plt.figure(figsize=(50, 20))
        
        # 设置大标题
        base_title = f'RX vs Strategy\nRX File: {os.path.basename(rx_csv_file)} | Strategy File: {os.path.basename(strategy_csv_file)}'
        
        fig_imu.suptitle(f'IMU Quaternion Comparison\n{base_title}', 
                        fontsize=16, fontweight='bold', y=0.98)
        
        fig_pos.suptitle(f'Motor Position Comparison\n{base_title}', 
                        fontsize=16, fontweight='bold', y=0.98)
        
        fig_vel.suptitle(f'Motor Velocity Comparison\n{base_title}', 
                        fontsize=16, fontweight='bold', y=0.98)
        
        # ============ 绘制 IMU 四元数对比图 ============
        print("\n" + "="*60)
        print("绘制IMU四元数对比图")
        print("="*60)
        
        # IMU四元数名称和单位
        imu_components = ['w', 'x', 'y', 'z']
        imu_titles = ['IMU Quaternion w', 'IMU Quaternion x', 'IMU Quaternion y', 'IMU Quaternion z']
        colors_imu = ['red', 'blue', 'green', 'orange']
        
        # 创建4行1列的IMU子图
        gs_imu = GridSpec(4, 1, figure=fig_imu, hspace=0.4, wspace=0.3)
        
        for i, component in enumerate(imu_components):
            ax_imu = fig_imu.add_subplot(gs_imu[i, 0])
            
            # 获取列名
            imu_col_rx = f'imu_{component}'
            imu_col_strategy = f'imu_{component}'
            
            if imu_col_rx in df_rx.columns and imu_col_strategy in df_strategy.columns:
                # 绘制RX数据
                ax_imu.plot(time_rx, df_rx[imu_col_rx], 
                          linewidth=1.5, color=colors_imu[i], alpha=0.8, 
                          label=f'RX {component}')
                
                # 绘制Strategy数据
                ax_imu.plot(time_strategy, df_strategy[imu_col_strategy], 
                          linewidth=1.5, color=colors_imu[i], linestyle='--', alpha=0.8, 
                          label=f'Strategy {component}')
                
                ax_imu.set_title(imu_titles[i], fontsize=14, fontweight='bold')
                ax_imu.set_xlabel('Time (s)', fontsize=12)
                ax_imu.set_ylabel('Value', fontsize=12)
                ax_imu.legend(loc='best', fontsize=10)
                ax_imu.grid(True, alpha=0.3)
                
                # 计算并显示最大值和最小值
                rx_max = df_rx[imu_col_rx].max()
                rx_min = df_rx[imu_col_rx].min()
                strategy_max = df_strategy[imu_col_strategy].max()
                strategy_min = df_strategy[imu_col_strategy].min()
                
                stats_text = f'RX: max={rx_max:.6f}, min={rx_min:.6f}\n'
                stats_text += f'Strategy: max={strategy_max:.6f}, min={strategy_min:.6f}'
                
                # 将统计信息放在左上角
                ax_imu.text(0.02, 0.98, stats_text, 
                           transform=ax_imu.transAxes, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                           fontsize=9)
                
                print(f"✓ 已绘制 IMU {component} 对比图")
            else:
                # 如果列不存在，显示错误信息
                ax_imu.text(0.5, 0.5, f'数据列缺失: {imu_col_rx}', 
                           ha='center', va='center', transform=ax_imu.transAxes,
                           fontsize=12)
                ax_imu.set_title(imu_titles[i] + ' (数据缺失)', fontsize=14)
                ax_imu.grid(True, alpha=0.3)
                print(f"✗ 列 {imu_col_rx} 在某个文件中不存在")
        
        # ============ 绘制电机位置对比图 ============
        print("\n" + "="*60)
        print("绘制电机位置对比图")
        print("="*60)
        
        # 创建6行1列的位置子图
        gs_pos = GridSpec(6, 1, figure=fig_pos, hspace=0.4, wspace=0.3)
        pos_axes = []
        
        for i in range(6):
            ax_pos = fig_pos.add_subplot(gs_pos[i, 0])
            pos_axes.append(ax_pos)
        
        # 电机索引和颜色
        motor_indices = range(1, 7)
        colors_pos = ['red', 'blue', 'green', 'orange', 'purple', 'brown']
        
        for i, motor_idx in enumerate(motor_indices):
            ax_pos = pos_axes[i]
            
            # 获取列名
            pos_col_rx = f'pos{motor_idx}'
            pos_col_strategy = f'pos{motor_idx}'
            
            if pos_col_rx in df_rx.columns and pos_col_strategy in df_strategy.columns:
                # 绘制RX数据
                ax_pos.plot(time_rx, df_rx[pos_col_rx], 
                          linewidth=1.5, color=colors_pos[i], alpha=0.8, 
                          label='RX Position')
                
                # 绘制Strategy数据
                ax_pos.plot(time_strategy, df_strategy[pos_col_strategy], 
                          linewidth=1.5, color=colors_pos[i], linestyle='--', alpha=0.8, 
                          label='Strategy Position')
                
                ax_pos.set_title(f'Motor {motor_idx} Position', fontsize=14, fontweight='bold')
                ax_pos.set_xlabel('Time (s)', fontsize=12)
                ax_pos.set_ylabel('Position', fontsize=12)
                ax_pos.legend(loc='best', fontsize=10)
                ax_pos.grid(True, alpha=0.3)
                
                # 计算并显示最大值和最小值
                rx_pos_max = df_rx[pos_col_rx].max()
                rx_pos_min = df_rx[pos_col_rx].min()
                strategy_pos_max = df_strategy[pos_col_strategy].max()
                strategy_pos_min = df_strategy[pos_col_strategy].min()
                
                stats_text = f'RX: max={rx_pos_max:.3f}, min={rx_pos_min:.3f}\n'
                stats_text += f'Strategy: max={strategy_pos_max:.3f}, min={strategy_pos_min:.3f}'
                
                ax_pos.text(0.02, 0.98, stats_text, 
                           transform=ax_pos.transAxes, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                           fontsize=9)
                
                print(f"✓ 已绘制 Motor {motor_idx} 位置对比图")
            else:
                ax_pos.text(0.5, 0.5, f'数据列缺失: pos{motor_idx}', 
                           ha='center', va='center', transform=ax_pos.transAxes,
                           fontsize=12)
                ax_pos.set_title(f'Motor {motor_idx} Position (数据缺失)', fontsize=14)
                ax_pos.grid(True, alpha=0.3)
                print(f"✗ 列 pos{motor_idx} 在某个文件中不存在")
        
        # ============ 绘制电机速度对比图 ============
        print("\n" + "="*60)
        print("绘制电机速度对比图")
        print("="*60)
        
        # 创建6行1列的速度子图
        gs_vel = GridSpec(6, 1, figure=fig_vel, hspace=0.4, wspace=0.3)
        vel_axes = []
        
        for i in range(6):
            ax_vel = fig_vel.add_subplot(gs_vel[i, 0])
            vel_axes.append(ax_vel)
        
        # 使用相同的颜色方案
        colors_vel = ['red', 'blue', 'green', 'orange', 'purple', 'brown']
        
        for i, motor_idx in enumerate(motor_indices):
            ax_vel = vel_axes[i]
            
            # 获取列名
            vel_col_rx = f'vel{motor_idx}'
            vel_col_strategy = f'vel{motor_idx}'
            
            if vel_col_rx in df_rx.columns and vel_col_strategy in df_strategy.columns:
                # 绘制RX数据
                ax_vel.plot(time_rx, df_rx[vel_col_rx], 
                          linewidth=1.5, color=colors_vel[i], alpha=0.8, 
                          label='RX Velocity')
                
                # 绘制Strategy数据
                ax_vel.plot(time_strategy, df_strategy[vel_col_strategy], 
                          linewidth=1.5, color=colors_vel[i], linestyle='--', alpha=0.8, 
                          label='Strategy Velocity')
                
                ax_vel.set_title(f'Motor {motor_idx} Velocity', fontsize=14, fontweight='bold')
                ax_vel.set_xlabel('Time (s)', fontsize=12)
                ax_vel.set_ylabel('Velocity', fontsize=12)
                ax_vel.legend(loc='best', fontsize=10)
                ax_vel.grid(True, alpha=0.3)
                
                # 计算并显示最大值和最小值
                rx_vel_max = df_rx[vel_col_rx].max()
                rx_vel_min = df_rx[vel_col_rx].min()
                strategy_vel_max = df_strategy[vel_col_strategy].max()
                strategy_vel_min = df_strategy[vel_col_strategy].min()
                
                stats_text = f'RX: max={rx_vel_max:.3f}, min={rx_vel_min:.3f}\n'
                stats_text += f'Strategy: max={strategy_vel_max:.3f}, min={strategy_vel_min:.3f}'
                
                ax_vel.text(0.02, 0.98, stats_text, 
                           transform=ax_vel.transAxes, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                           fontsize=9)
                
                print(f"✓ 已绘制 Motor {motor_idx} 速度对比图")
            else:
                ax_vel.text(0.5, 0.5, f'数据列缺失: vel{motor_idx}', 
                           ha='center', va='center', transform=ax_vel.transAxes,
                           fontsize=12)
                ax_vel.set_title(f'Motor {motor_idx} Velocity (数据缺失)', fontsize=14)
                ax_vel.grid(True, alpha=0.3)
                print(f"✗ 列 vel{motor_idx} 在某个文件中不存在")
        
        # 调整布局
        fig_imu.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig_pos.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig_vel.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # 询问是否保存图片
        print("\n" + "="*60)
        print("图片保存选项")
        print("="*60)

        save_choice = input("是否保存三张图片？(y/n): ").lower()
        if save_choice == 'y':
            # 固定保存文件夹
            save_folder = "../figures_data/figures"
            os.makedirs(save_folder, exist_ok=True)
            
            print(f"\n图片将保存到: {save_folder}")
            print("只需输入文件名，无需输入完整路径（会自动添加.png扩展名）\n")
            
            # 询问文件名
            imu_name = input("请输入IMU图文件名: ").strip()
            pos_name = input("请输入位置图文件名: ").strip()
            vel_name = input("请输入速度图文件名: ").strip()
            
            # 确保有文件名
            base = os.path.basename(rx_csv_file).split('.')[0]
            
            imu_name = imu_name if imu_name else f"imu_comparison_{base}"
            pos_name = pos_name if pos_name else f"pos_comparison_{base}"
            vel_name = vel_name if vel_name else f"vel_comparison_{base}"
            
            # 添加扩展名
            for name in [imu_name, pos_name, vel_name]:
                if not name.lower().endswith('.png'):
                    name += '.png'
            
            # 构建完整路径
            imu_path = os.path.join(save_folder, imu_name)
            pos_path = os.path.join(save_folder, pos_name)
            vel_path = os.path.join(save_folder, vel_name)
            
            # 保存
            fig_imu.savefig(imu_path, dpi=300, bbox_inches='tight')
            fig_pos.savefig(pos_path, dpi=300, bbox_inches='tight')
            fig_vel.savefig(vel_path, dpi=300, bbox_inches='tight')
            
            print(f"\n保存完成！")
            print(f"IMU图: {imu_path}")
            print(f"位置图: {pos_path}")
            print(f"速度图: {vel_path}")
        else:
            print("跳过保存图片")

        # 显示图片
        print("\n显示IMU四元数对比图...")
        plt.figure(fig_imu.number)
        plt.show(block=False)
        
        print("显示电机位置对比图...")
        plt.figure(fig_pos.number)
        plt.show(block=False)
        
        print("显示电机速度对比图...")
        plt.figure(fig_vel.number)
        plt.show(block=True)
        
        print("\n对比绘图完成")
        
    except Exception as e:
        print(f"处理文件时出错: {e}")
        import traceback
        traceback.print_exc()

def main():
    """
    主函数 - 程序入口
    """
    print("=" * 60)
    print("          数据对比绘图工具（RX vs Strategy）")
    print("          三张图：IMU四元数 | 电机位置 | 电机速度")
    print("          多行一列布局 | 只显示最大最小值")
    print("=" * 60)
    
    # 获取文件路径
    rx_file_path = input("请输入RX数据CSV文件路径: ").strip()
    strategy_file_path = input("请输入Strategy数据CSV文件路径: ").strip()
    
    # 设置默认文件名
    if not rx_file_path:
        rx_file_path = "../figures_data/data/log_strategy_0.csv"  # 读取文件路径
        print(f"使用默认RX数据文件: {rx_file_path}")
    
    if not strategy_file_path:
        # strategy_file_path = "../figures_data/data/obs_env_csv1219/v2_obs_imuposvel.csv"  # 读取文件路径
        strategy_file_path = "../figures_data/data/log_rx_505.csv"  # 读取文件路径
        print(f"使用默认Strategy数据文件: {strategy_file_path}")
    
    # 运行对比绘图函数
    plot_motor_data_comparison(rx_file_path, strategy_file_path)
    
    # 询问是否继续
    while True:
        continue_choice = input("\n是否继续绘制其他文件对？(y/n): ").lower()
        if continue_choice == 'y':
            new_rx_file = input("请输入新的RX数据CSV文件路径: ").strip()
            new_strategy_file = input("请输入新的Strategy数据CSV文件路径: ").strip()
            plot_motor_data_comparison(new_rx_file, new_strategy_file)
        elif continue_choice == 'n':
            print("程序结束，再见！")
            break
        else:
            print("请输入 y 或 n")

# 程序入口
if __name__ == "__main__":
    main()